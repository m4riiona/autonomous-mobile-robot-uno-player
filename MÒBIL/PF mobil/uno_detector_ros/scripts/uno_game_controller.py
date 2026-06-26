#!/usr/bin/env python3

import json
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import rospy
from std_msgs.msg import String


GAME_DELAY = 4.0  # seconds to lock detection after the robot plays

COLORS = ("RED", "GREEN", "BLUE", "YELLOW")
RANKS = (
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    "SKIP", "REVERSE", "DRAW2", "DRAW4", "WILD",
)

# Abbreviated or alternate rank tokens the model may emit
_RANK_ALIASES: Dict[str, str] = {
    "REV": "REVERSE",
    "SKP": "SKIP",
    "BLOCK": "SKIP",
    "BLOCKED": "SKIP",
    "BLOQUEO": "SKIP",
    "D2": "DRAW2",
    "PLUS2": "DRAW2",
    "+2": "DRAW2",
    "D4": "DRAW4",
    "PLUS4": "DRAW4",
    "+4": "DRAW4",
    "W": "WILD",
}


@dataclass
class ParsedCard:
    raw_label: str
    color: Optional[str]
    rank: Optional[str]
    confidence: float = 0.0

    @classmethod
    def from_detection(cls, detection: Dict[str, Any]) -> "ParsedCard":
        label = str(detection.get("label", "")).strip()
        confidence = float(detection.get("confidence", 0.0) or 0.0)
        return cls.from_label(label, confidence)

    @classmethod
    def from_label(cls, label: str, confidence: float = 0.0) -> "ParsedCard":
        # Normalizamos nombres del detector: RED_5, BLOQUEO, +4, WILD_DRAW_FOUR, etc.
        normalized = label.strip().upper().replace("_", " ").replace("-", " ").replace("/", " ")
        normalized = normalized.replace("+ 4", "+4").replace("+ 2", "+2")
        normalized = " ".join(normalized.split())
        normalized = normalized.replace("DRAW FOUR", "DRAW4").replace("DRAW TWO", "DRAW2")
        normalized = normalized.replace("DRAW 4", "DRAW4").replace("DRAW 2", "DRAW2")
        normalized = normalized.replace("WILD DRAW4", "DRAW4")
        tokens = normalized.split()

        color = next((token for token in tokens if token in COLORS), None)
        rank = None

        # Exact token match first, then alias lookup per token
        for token in tokens:
            if token in RANKS:
                rank = token
                break
            if token in _RANK_ALIASES:
                rank = _RANK_ALIASES[token]
                break

        # Fuzzy fallback: substring match against the full normalised string
        if rank is None:
            for candidate in RANKS:
                if candidate in normalized:
                    rank = candidate
                    break

        return cls(raw_label=label, color=color, rank=rank, confidence=confidence)

    @property
    def is_wild(self) -> bool:
        return self.rank in {"DRAW4", "WILD"}

    def can_play_on(self, top_card: Optional["ParsedCard"]) -> bool:
        if top_card is None:
            return True
        if self.is_wild:
            return True
        # Wild on top with no declared color (opponent played it) — anything goes
        if top_card.is_wild and top_card.color is None:
            return True
        if self.color is not None and top_card.color is not None and self.color == top_card.color:
            return True
        if self.rank is not None and top_card.rank is not None and self.rank == top_card.rank:
            return True
        return False

    def __str__(self) -> str:
        if self.color and self.rank:
            return f"{self.color} {self.rank}"
        return self.raw_label or "UNKNOWN"


class UnoGameController:
    def __init__(self) -> None:
        self.detections_topic = rospy.get_param("~detections_topic", "/uno_detector/detections")
        self.initial_cards_param = rospy.get_param("~initial_cards", None)
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 15.0))
        self.state_topic = rospy.get_param("~state_topic", "~state")

        # ── Synchronisation primitives ──────────────────────────────────────
        self._condition = threading.Condition()
        self._message_seq: int = 0
        self._latest_payload: Optional[Dict[str, Any]] = None

        # Deduplication state:
        #   _last_accepted_label      – normalised label of the last card added to hand
        #   _consecutive_empty_frames – consecutive frames with no detection since the last
        #                               accepted card; must reach 10 before the same card
        #                               label is accepted again
        self._last_accepted_label: Optional[str] = None
        self._consecutive_empty_frames: int = 10  # starts at 10 so the very first card is always accepted

        # Phase: "scan"           → collecting the robot's initial 7 cards
        #        "waiting_start"  → hand full, waiting for Enter to begin
        #        "game"           → each detection is the opponent's played card
        #        "robot_drawing"  → robot drawing cards; camera scans each one
        self._phase: str = "scan"

        # Drawing state
        self._cards_to_draw: int = 0
        self._draw_may_play: bool = True  # False when penalised by Draw2/Draw4

        # Time gate: detections in "game" phase are ignored until this timestamp
        self._accept_after: float = 0.0

        # ── Game state ───────────────────────────────────────────────────────
        self.robot_hand: List[ParsedCard] = []
        self.top_card: Optional[ParsedCard] = None
        self.last_opponent_card: Optional[ParsedCard] = None

        # ── ROS I/O ──────────────────────────────────────────────────────────
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=10)
        self.detection_sub = rospy.Subscriber(
            self.detections_topic, String, self._detections_cb, queue_size=10
        )

        rospy.loginfo("UNO controller subscribed to %s", self.detections_topic)

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _normalise_label(self, label: str) -> str:
        return " ".join(label.strip().upper().split())

    def _detections_cb(self, msg: String) -> None:
        """Process every incoming detection frame.

        Acceptance rules:
          - Confidence must be > 0.8.
          - To re-accept the same card label, 10 consecutive empty frames must
            have elapsed since it was last accepted (card physically removed).
          - At most 7 cards are added to the hand (scan phase cap).
        """
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logwarn("UNO controller received invalid detection payload")
            return

        with self._condition:
            self._message_seq += 1
            self._latest_payload = payload
            self._condition.notify_all()

        count = int(payload.get("count", 0))

        # Empty frame: count the gap
        detections = payload.get("detections", []) if count > 0 else []
        if not detections:
            self._consecutive_empty_frames += 1
            return

        # Non-empty frame — shared pre-checks for both phases
        best = max(detections, key=lambda d: float(d.get("confidence", 0.0) or 0.0))
        card = ParsedCard.from_detection(best)

        gap_before = self._consecutive_empty_frames
        self._consecutive_empty_frames = 0

        # Confidence gate
        if card.confidence <= 0.8:
            return

        norm = self._normalise_label(card.raw_label)

        # Duplicate guard: need 10 consecutive empty frames before re-accepting same card
        if norm == self._last_accepted_label and gap_before < 10:
            return

        self._last_accepted_label = norm

        # ── Scan phase: build the initial 7-card hand ─────────────────────────
        if self._phase == "scan":
            rospy.loginfo("Card detected and added to hand: %s (conf=%.2f)  [%d/7]",
                          card, card.confidence, len(self.robot_hand) + 1)
            self.robot_hand.append(card)

            if len(self.robot_hand) >= 7:
                rospy.loginfo("")
                rospy.loginfo("Hand is now full. Cards scanned:")
                for i, c in enumerate(self.robot_hand, 1):
                    rospy.loginfo("  %d. %s", i, c)
                rospy.loginfo("")
                rospy.loginfo("Press ENTER to start the game (robot goes first)...")
                self._phase = "waiting_start"
                self._last_accepted_label = None
                self._consecutive_empty_frames = 10
                self._publish_state("scan_complete", "hand full – press Enter to start")
                threading.Thread(target=self._wait_for_start, daemon=True).start()
            else:
                self._publish_state("scan", f"auto-scanned {card}")

        # ── Robot drawing phase: camera scans each drawn card ─────────────────
        elif self._phase == "robot_drawing":
            rospy.loginfo("Robot draws: %s (conf=%.2f)  [%d card(s) left to draw]",
                          card, card.confidence, self._cards_to_draw - 1)
            self.robot_hand.append(card)
            self._cards_to_draw -= 1

            if self._cards_to_draw <= 0:
                self._phase = "game"
                if self._draw_may_play:
                    rospy.loginfo("")
                    rospy.loginfo("--- ROBOT'S TURN (after draw) ---")
                    self._auto_play()
                if self.robot_hand:
                    self._accept_after = time.time() + GAME_DELAY
                    rospy.loginfo("")
                    rospy.loginfo("--- OPPONENT'S TURN – Show your card to the camera ---")

        # ── Game phase: each detection is the opponent's played card ──────────
        elif self._phase == "game":
            # Time gate: cards are ignored until GAME_DELAY seconds after the last
            # accepted card (set immediately after recognition below)
            if time.time() < self._accept_after:
                return

            # Validate: the opponent's card must be a legal play on the current top card
            if not card.can_play_on(self.top_card):
                rospy.logwarn(
                    "Invalid play detected: %s cannot be played on %s – ignoring",
                    card, self.top_card
                )
                return

            rospy.loginfo("")
            rospy.loginfo("--- OPPONENT'S TURN ---")
            rospy.loginfo("Opponent played: %s (conf=%.2f)", card, card.confidence)
            self.top_card = card
            self.last_opponent_card = card
            self._publish_state("opponent_play", str(card))

            # Lock the gate NOW (after recognition) so the same physical card
            # cannot be re-scanned for the next GAME_DELAY seconds.
            # _last_accepted_label is intentionally kept as the opponent's card
            # label so the dedup also blocks it in robot_drawing phase.
            self._accept_after = time.time() + GAME_DELAY

            if card.rank == "DRAW2":
                rospy.loginfo(">>> Robot must draw 2 cards! Show them to the camera one at a time. <<<")
                self._phase = "robot_drawing"
                self._cards_to_draw = 2
                self._draw_may_play = False
                # Do NOT reset dedup state: opponent's card stays as _last_accepted_label
                # so it cannot be immediately re-detected in robot_drawing phase
                return
            if card.rank == "DRAW4":
                rospy.loginfo(">>> Robot must draw 4 cards! Show them to the camera one at a time. <<<")
                self._phase = "robot_drawing"
                self._cards_to_draw = 4
                self._draw_may_play = False
                return

            rospy.loginfo("")
            rospy.loginfo("--- ROBOT'S TURN ---")
            self._auto_play()
            if self.robot_hand:
                rospy.loginfo("")
                rospy.loginfo("--- OPPONENT'S TURN – Show your card to the camera ---")

    def _wait_for_start(self) -> None:
        """Blocks in a daemon thread until Enter is pressed, then starts the game."""
        try:
            input()
        except EOFError:
            pass
        rospy.loginfo("")
        rospy.loginfo("=" * 55)
        rospy.loginfo("  GAME STARTED – Robot goes first!")
        rospy.loginfo("=" * 55)
        self._phase = "game"
        rospy.loginfo("")
        rospy.loginfo("--- ROBOT'S TURN ---")
        self._auto_play()
        if self.robot_hand:
            self._accept_after = time.time() + GAME_DELAY
            rospy.loginfo("")
            rospy.loginfo("--- OPPONENT'S TURN – Show your card to the camera ---")

    def _best_color(self) -> str:
        """Return the color most represented in the robot's hand (excluding wilds)."""
        counts: Dict[str, int] = {c: 0 for c in COLORS}
        for card in self.robot_hand:
            if card.color in counts:
                counts[card.color] += 1
        return max(counts, key=lambda c: counts[c])

    def _auto_play(self) -> None:
        """Pick the best playable card from the robot's hand and play it automatically.

        Nueva prioridad de decision:
          1. +4 siempre primero.
          2. Cartas especiales del color activo antes que numeros.
          3. Logica normal del proyecto si no aplica lo anterior.
        top_card=None means it is the very first move; all cards are valid.
        """
        # 1) Prioridad absoluta: si existe un +4 en la mano, se juega
        # independientemente de la carta superior o del color activo.
        draw4_cards = [c for c in self.robot_hand if c.rank == "DRAW4"]
        if draw4_cards:
            chosen = max(draw4_cards, key=lambda c: c.confidence)
        else:
            # Mantenemos el caso especial de SKIP/REVERSE: el robot pierde turno
            # salvo la excepcion anterior del +4 absoluto.
            if self.top_card is not None and self.top_card.rank in {"SKIP", "REVERSE"}:
                rospy.loginfo(">>> Robot loses its turn after %s. <<<", self.top_card.rank)
                self._publish_state("no_play", f"turn skipped by {self.top_card.rank}")
                return

            # Partition playable cards (None top_card = first move, everything is valid)
            playable = [c for c in self.robot_hand if c.can_play_on(self.top_card)]
            if not playable:
                rospy.loginfo(">>> Robot has no playable card – drawing 1 card from the deck. <<<")
                rospy.loginfo("Show the drawn card to the camera.")
                self._phase = "robot_drawing"
                self._cards_to_draw = 1
                self._draw_may_play = True
                # Keep dedup state: opponent must remove their card before the
                # drawn card can be scanned (prevents their card being added to hand)
                self._publish_state("no_play", "no playable card – drawing")
                return

            active_color = self.top_card.color if self.top_card is not None else None

            # 2) Si hay cartas especiales del mismo color activo, las jugamos
            # antes que una carta numerica de ese color.
            same_color_actions = [
                c for c in playable
                if active_color is not None and c.color == active_color and c.rank in {"SKIP", "REVERSE", "DRAW2"}
            ]
            if same_color_actions:
                chosen = max(same_color_actions, key=lambda c: c.confidence)
            else:
                # 3) Si no aplica ninguna regla nueva, se conserva la prioridad
                # normal: acciones no wild > numeros > wild.
                def priority(c: ParsedCard) -> int:
                    if c.is_wild:
                        return 0
                    if c.cardtype_str == "action":
                        return 2
                    return 1

                # Attach a lightweight type hint so priority() can inspect it
                for c in playable:
                    if not hasattr(c, "cardtype_str"):
                        if c.rank in {"SKIP", "REVERSE", "DRAW2"}:
                            object.__setattr__(c, "cardtype_str", "action")
                        elif c.is_wild:
                            object.__setattr__(c, "cardtype_str", "wild")
                        else:
                            object.__setattr__(c, "cardtype_str", "number")

                chosen = max(playable, key=priority)

        # Remove from hand
        self._remove_card_from_hand(chosen)

        # For wilds, pick the most common color in remaining hand
        if chosen.is_wild:
            chosen.color = self._best_color()
            rospy.loginfo("Robot picks color: %s", chosen.color)

        self.top_card = chosen
        self._publish_state("play", str(chosen))
        rospy.loginfo("Robot plays: %s  |  %d card(s) remaining in hand", chosen, len(self.robot_hand))

        if chosen.rank == "DRAW2":
            rospy.loginfo(">>> Opponent must draw 2 cards! <<<")
        elif chosen.rank == "DRAW4":
            rospy.loginfo(">>> Opponent must draw 4 cards! <<<")

        if len(self.robot_hand) == 0:
            rospy.loginfo("")
            rospy.loginfo("*** ROBOT WINS – hand is empty! ***")
            self._publish_state("win", "robot hand empty")

    def _publish_state(self, event: str, note: str = "") -> None:
        payload: Dict[str, Any] = {
            "event": event,
            "note": note,
            "hand": [str(card) for card in self.robot_hand],
        }
        if self.top_card is not None:
            payload["top_card"] = str(self.top_card)
        if self.last_opponent_card is not None:
            payload["last_opponent_card"] = str(self.last_opponent_card)
        self.state_pub.publish(String(data=json.dumps(payload)))

    def _remove_card_from_hand(self, target: ParsedCard) -> bool:
        for index, card in enumerate(self.robot_hand):
            if card.raw_label.strip().upper() == target.raw_label.strip().upper():
                self.robot_hand.pop(index)
                return True
        return False

    def _show_summary(self) -> None:
        rospy.loginfo("Robot hand (%d cards):", len(self.robot_hand))
        for i, card in enumerate(self.robot_hand, start=1):
            rospy.loginfo("  %d. %s", i, card)
        rospy.loginfo("Top card: %s", self.top_card if self.top_card else "unknown")

    # ── Opponent-play helper (still useful to record opponent moves) ──────────

    def _record_opponent_play(self, label: str) -> None:
        card = ParsedCard.from_label(label)
        self.last_opponent_card = card
        self.top_card = card
        self._publish_state("opponent_play", str(card))
        rospy.loginfo("Opponent played: %s", card)

    # ── Main entry point ──────────────────────────────────────────────────────

    def run(self) -> None:
        rospy.loginfo("UNO controller ready. Waiting for card detections...")
        rospy.loginfo(
            "Cards are added automatically as soon as the detector sees them. "
            "The robot will play autonomously after each new card is added."
        )

        # Allow an operator to register the initial top card via a ROS param
        initial_top = rospy.get_param("~initial_top_card", None)
        if initial_top:
            self.top_card = ParsedCard.from_label(str(initial_top))
            rospy.loginfo("Initial top card set from param: %s", self.top_card)
            self._publish_state("initialized", f"top card: {self.top_card}")

        # Spin — all logic is driven by _detections_cb
        rospy.spin()


def main() -> None:
    rospy.init_node("uno_game_controller")
    controller = UnoGameController()
    try:
        controller.run()
    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == "__main__":
    main()
