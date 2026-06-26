#!/usr/bin/env python3

import json
import os
import re
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from turtlebot3_msgs.msg import Sound


COLORS = ("RED", "GREEN", "BLUE", "YELLOW")
NUMBER_RANKS = tuple(str(i) for i in range(10))
ACTION_RANKS = ("DRAW2", "SKIP", "REVERSE")
SPECIAL_RANKS = ("WILD", "DRAW4")
ALL_RANKS = NUMBER_RANKS + ACTION_RANKS + SPECIAL_RANKS

RANK_ALIASES = {
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
    "DRAW":"DRAW2"
}


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


@dataclass
class UnoCard:
    raw_label: str
    color: Optional[str]
    rank: Optional[str]
    confidence: float = 0.0

    @staticmethod
    def _normalise_label(label: str) -> str:
        text = str(label).strip().upper()
        text = text.replace("_", " ").replace("-", " ").replace("/", " ")
        text = text.replace("+ 4", "+4").replace("+ 2", "+2")
        text = re.sub(r"\bDRAW\s*FOUR\b", "DRAW4", text)
        text = re.sub(r"\bDRAW\s*TWO\b", "DRAW2", text)
        text = re.sub(r"\bDRAW\s*4\b", "DRAW4", text)
        text = re.sub(r"\bDRAW\s*2\b", "DRAW2", text)
        text = re.sub(r"\bWILD\s*DRAW4\b", "DRAW4", text)
        text = re.sub(r"\bWILD\s*DRAW\s*4\b", "DRAW4", text)
        return " ".join(text.split())

    @classmethod
    def from_label(cls, label: str, confidence: float = 0.0) -> "UnoCard":
        normalised = cls._normalise_label(label)
        tokens = normalised.split()

        color = next((token for token in tokens if token in COLORS), None)
        rank = None

        for token in tokens:
            if token in ALL_RANKS:
                rank = token
                break
            if token in RANK_ALIASES:
                rank = RANK_ALIASES[token]
                break

        # Fallback sencillo por si el modelo publica algo como RED5 o YELLOWREVERSE.
        if rank is None:
            for candidate in ("DRAW4", "DRAW2", "REVERSE", "SKIP", "WILD") + NUMBER_RANKS:
                if candidate in normalised:
                    rank = candidate
                    break

        return cls(raw_label=str(label), color=color, rank=rank, confidence=float(confidence))

    @classmethod
    def from_detection(cls, detection: Dict[str, Any]) -> "UnoCard":
        label = str(detection.get("label", "")).strip()
        confidence = float(detection.get("confidence", 0.0) or 0.0)
        return cls.from_label(label, confidence)

    @property
    def is_valid(self) -> bool:
        if self.rank in SPECIAL_RANKS:
            return True
        return self.color in COLORS and self.rank in ALL_RANKS

    @property
    def is_wild_or_draw4(self) -> bool:
        return self.rank in SPECIAL_RANKS

    def key(self) -> str:
        return self._normalise_label(str(self))

    def __str__(self) -> str:
        if self.rank in SPECIAL_RANKS and self.color is None:
            return self.rank
        if self.color and self.rank:
            return f"{self.color} {self.rank}"
        return self.raw_label or "UNKNOWN"


@dataclass
class DetectedCard:
    card: UnoCard
    stamp: float


@dataclass
class ScannedOption:
    waypoint_index: int
    waypoint: Waypoint
    card: Optional[UnoCard]
    confidence: float = 0.0


class UnoOrchestrator:
    def __init__(self) -> None:
        rospy.init_node("uno_orchestrator")

        self.waypoints_file = rospy.get_param(
            "~waypoints_file",
            "$(find uno_wall_navigation)/config/card_waypoints.yaml",
        )
        self.detections_topic = rospy.get_param("~detections_topic", "/uno_detector/detections")
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.70))
        self.move_base_timeout = float(rospy.get_param("~move_base_timeout", 90.0))
        self.sound_enabled = bool(rospy.get_param("~sound_enabled", True))
        self.scan_each_card_timeout = float(rospy.get_param("~scan_each_card_timeout", 5.0))
        self.debug = bool(rospy.get_param("~debug", True))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.stop_delay = float(rospy.get_param("~stop_delay", 0.7))

        self.waypoints = self.load_waypoints(self.waypoints_file)

        self._condition = threading.Condition()
        self._detection_buffer: List[DetectedCard] = []
        self._last_payload_error_printed = False
        self._sound_warning_printed = False

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sound_pub = rospy.Publisher("/sound", Sound, queue_size=1)
        self.status_pub = rospy.Publisher("/uno_orchestrator/status", String, queue_size=10, latch=True)

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.detections_sub = rospy.Subscriber(
            self.detections_topic,
            String,
            self.detections_callback,
            queue_size=20,
        )

        rospy.on_shutdown(self.on_shutdown)

        self.publish_status("inicializando")
        rospy.loginfo("Orquestador UNO inicializado con %d waypoints", len(self.waypoints))
        rospy.loginfo("Topic de detecciones: %s", self.detections_topic)
        rospy.loginfo("Confianza minima: %.2f", self.min_confidence)

    # ------------------------------------------------------------------
    # Carga de waypoints y utilidades ROS
    # ------------------------------------------------------------------

    def load_waypoints(self, path: str) -> List[Waypoint]:
        if "$(find" in path:
            rospy.logwarn(
                "El parametro waypoints_file no parece resuelto por roslaunch: %s. "
                "Lanza el nodo con el launch del paquete.",
                path,
            )

        expanded_path = os.path.expanduser(path)
        if not os.path.exists(expanded_path):
            raise rospy.ROSException(f"No existe el YAML de waypoints: {expanded_path}")

        with open(expanded_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        raw_waypoints = data.get("waypoints", [])
        waypoints: List[Waypoint] = []
        for i, entry in enumerate(raw_waypoints):
            try:
                waypoints.append(
                    Waypoint(
                        name=str(entry.get("name", f"carta_{i + 1}")),
                        x=float(entry["x"]),
                        y=float(entry["y"]),
                        yaw=float(entry.get("yaw", 0.0)),
                    )
                )
            except Exception as exc:
                rospy.logwarn("Waypoint ignorado por formato incorrecto: %s (%s)", entry, exc)

        if not waypoints:
            raise rospy.ROSException("El YAML no contiene waypoints validos.")

        return waypoints

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def on_shutdown(self) -> None:
        try:
            self.move_base.cancel_goal()
        except Exception:
            pass
        self.stop_robot()

    def play_sound(self, value: int) -> None:
        if not self.sound_enabled:
            return

        if self.sound_pub.get_num_connections() == 0 and not self._sound_warning_printed:
            rospy.logwarn("/sound no tiene subscribers ahora mismo. Publico igualmente y continuo.")
            self._sound_warning_printed = True

        try:
            msg = Sound()
            msg.value = int(value)
            self.sound_pub.publish(msg)
        except Exception as exc:
            rospy.logwarn("No se pudo publicar sonido %s: %s", value, exc)

    # ------------------------------------------------------------------
    # Detecciones de YOLO
    # ------------------------------------------------------------------

    def detections_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])
        except Exception as exc:
            if not self._last_payload_error_printed:
                rospy.logwarn("No puedo parsear JSON de detecciones: %s", exc)
                self._last_payload_error_printed = True
            return

        now = rospy.Time.now().to_sec()
        valid_cards: List[DetectedCard] = []

        for detection in detections:
            try:
                card = UnoCard.from_detection(detection)
            except Exception as exc:
                if self.debug:
                    rospy.logwarn("Deteccion ignorada por formato raro: %s (%s)", detection, exc)
                continue

            if card.confidence < self.min_confidence:
                continue
            if not card.is_valid:
                if self.debug:
                    rospy.logwarn("Etiqueta no reconocida como carta UNO: %s", card.raw_label)
                continue

            valid_cards.append(DetectedCard(card=card, stamp=now))

        if not valid_cards:
            return

        with self._condition:
            self._detection_buffer.extend(valid_cards)
            # No dejamos crecer la lista si el detector publica durante mucho rato.
            self._detection_buffer = self._detection_buffer[-100:]
            self._condition.notify_all()

    def wait_for_card(
        self,
        timeout: Optional[float] = None,
        wait_until_seen: bool = False,
    ) -> Optional[UnoCard]:
        """Recoge detecciones durante una ventana corta y devuelve la de mayor confianza."""
        scan_time = float(timeout if timeout is not None else self.scan_each_card_timeout)

        while not rospy.is_shutdown():
            with self._condition:
                self._detection_buffer = []

            deadline = rospy.Time.now().to_sec() + scan_time
            while not rospy.is_shutdown():
                remaining = deadline - rospy.Time.now().to_sec()
                if remaining <= 0.0:
                    break
                with self._condition:
                    self._condition.wait(timeout=min(remaining, 0.25))

            with self._condition:
                candidates = list(self._detection_buffer)

            if candidates:
                best = max(candidates, key=lambda item: item.card.confidence)
                return best.card

            if not wait_until_seen:
                return None

            rospy.logwarn("No se ha detectado una carta valida. Sigo esperando...")
            self.publish_status("esperando_carta")

        return None

    # ------------------------------------------------------------------
    # Navegacion con move_base
    # ------------------------------------------------------------------

    def ensure_move_base(self) -> bool:
        if self.move_base.wait_for_server(rospy.Duration(5.0)):
            return True
        rospy.logerr("move_base no responde. Comprueba que la navegacion esta lanzada.")
        return False

    def make_goal(self, waypoint: Waypoint) -> MoveBaseGoal:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y
        goal.target_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, waypoint.yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    def go_to_waypoint(self, waypoint: Waypoint) -> Tuple[bool, str]:
        if not self.ensure_move_base():
            return False, "move_base no disponible"

        rospy.loginfo(
            "Enviando goal a %s: x=%.2f y=%.2f yaw=%.2f",
            waypoint.name,
            waypoint.x,
            waypoint.y,
            waypoint.yaw,
        )
        self.publish_status(f"navegando_a_{waypoint.name}")

        self.move_base.send_goal(self.make_goal(waypoint))
        finished = self.move_base.wait_for_result(rospy.Duration(self.move_base_timeout))
        self.stop_robot()

        if not finished:
            self.move_base.cancel_goal()
            self.stop_robot()
            return False, "timeout"

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.sleep(self.stop_delay)
            self.stop_robot()
            return True, "ok"

        return False, f"estado move_base {state}"

    def small_camera_adjustment_and_retry(self) -> Optional[UnoCard]:
        rospy.logwarn("Intento un reajuste simple de camara con /cmd_vel.")
        self.publish_status("reajuste_camara")

        for angular_z, seconds in ((0.18, 1.0), (-0.18, 2.0), (0.18, 1.0)):
            twist = Twist()
            twist.angular.z = angular_z
            end_time = rospy.Time.now().to_sec() + seconds
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() < end_time:
                self.cmd_pub.publish(twist)
                rate.sleep()
            self.stop_robot()
            rospy.sleep(0.3)

            card = self.wait_for_card(timeout=max(1.5, self.scan_each_card_timeout / 2.0))
            if card is not None:
                return card

        self.stop_robot()
        return None

    def scan_all_waypoints(self) -> List[ScannedOption]:
        scanned: List[ScannedOption] = []

        for index, waypoint in enumerate(self.waypoints):
            if rospy.is_shutdown():
                break

            ok, detail = self.go_to_waypoint(waypoint)
            if not ok:
                rospy.logwarn("No se ha podido llegar a %s: %s", waypoint.name, detail)
                scanned.append(ScannedOption(index, waypoint, None, 0.0))
                continue

            self.publish_status(f"leyendo_{waypoint.name}")
            rospy.sleep(0.8)
            card = self.wait_for_card(timeout=self.scan_each_card_timeout)

            if card is None:
                rospy.logwarn("[%s] No se ha detectado carta. Reintentando con pequeno giro.", waypoint.name)
                card = self.small_camera_adjustment_and_retry()

            if card is None:
                rospy.logwarn("[%s] Sin deteccion final. Continuo con el siguiente waypoint.", waypoint.name)
                scanned.append(ScannedOption(index, waypoint, None, 0.0))
            else:
                rospy.loginfo("[%s] Detectada: %s conf=%.2f", waypoint.name, card, card.confidence)
                scanned.append(ScannedOption(index, waypoint, card, card.confidence))
                self.play_sound(4)

        return scanned

    # ------------------------------------------------------------------
    # Reglas simples de UNO
    # ------------------------------------------------------------------

    def ask_active_color(self, prompt: str) -> str:
        while not rospy.is_shutdown():
            try:
                answer = input(prompt).strip().upper()
            except EOFError:
                rospy.logwarn("No se puede leer input por terminal. Uso RED por defecto.")
                return "RED"

            if answer in COLORS:
                return answer
            print("Color no valido. Escribe RED, GREEN, BLUE o YELLOW.")

        return "RED"

    def is_playable(self, card: UnoCard, opponent_card: UnoCard, active_color: Optional[str]) -> bool:
        if card.rank in SPECIAL_RANKS:
            return True
        if active_color is not None and card.color == active_color:
            return True
        if opponent_card.rank not in SPECIAL_RANKS and card.rank == opponent_card.rank:
            return True
        return False

    def choose_card(
        self,
        opponent_card: UnoCard,
        active_color: Optional[str],
        scanned: List[ScannedOption],
    ) -> Optional[ScannedOption]:
        options = [item for item in scanned if item.card is not None and item.card.is_valid]


        # Caso A: +4
        # con otro +4. Si no hay +4, el robot no juega y espera.
        if opponent_card.rank == "DRAW4":
            draw4_options = [item for item in options if item.card.rank == "DRAW4"]
            if draw4_options:
                print("El oponente me lanzó un +4, ¡pero puedo devolver el ataque! :)")
                return max(draw4_options, key=lambda item: item.confidence)
            else:
                print("¡No tengo un +4 para defenderme! Robo 4 cartas y busco qué jugar...")

        # Caso B: El oponente jugó +2 (DRAW2)
        elif opponent_card.rank == "DRAW2":
            draw2_options = [item for item in options if item.card.rank == "DRAW2"]
            if draw2_options:
                print("¡Me lanzaron un +2! Por suerte tengo otro para devolver el ataque :).")
                return max(draw2_options, key=lambda item: item.confidence)
            else:
                print("¡No tengo un +2 para defenderme! Robo 2 cartas, pero voy a jugar una carta...")

        # Mantenemos el caso de SKIP/REVERSE: el robot pierde el turno, salvo
        # la excepcion anterior del +4 absoluto.
        elif opponent_card.rank in ("SKIP", "REVERSE"):
            return None

        playable = [item for item in options if self.is_playable(item.card, opponent_card, active_color)]
        if not playable:
            return None

        # 2) Si hay cartas jugables del color activo, preferimos especiales
        # del mismo color (DRAW2, SKIP/BLOQUEO, REVERSE) antes que numeros.
        same_color_actions = [
            item
            for item in playable
            if active_color is not None
            and item.card.color == active_color
            and item.card.rank in ACTION_RANKS
        ]
        if same_color_actions:
            return max(same_color_actions, key=lambda item: item.confidence)

        # 3) Si no se cumple ninguna regla nueva, conservamos la prioridad
        # original del proyecto para no cambiar el resto de comportamientos.
        def priority(item: ScannedOption) -> Tuple[int, float]:
            card = item.card
            if card.rank in ACTION_RANKS:
                group = 1
            elif opponent_card.rank not in SPECIAL_RANKS and card.rank == opponent_card.rank:
                group = 2
            elif active_color is not None and card.color == active_color:
                group = 3
            elif card.rank == "WILD":
                group = 4
            elif card.rank == "DRAW4":
                group = 5
            else:
                group = 99
            return (group, -item.confidence)

        return min(playable, key=priority)

    def choose_color_from_seen_cards(self, scanned: List[ScannedOption]) -> str:
        counts = {color: 0 for color in COLORS}
        for item in scanned:
            if item.card is not None and item.card.color in counts:
                counts[item.card.color] += 1

        best_color = max(COLORS, key=lambda color: counts[color])
        if counts[best_color] == 0:
            return "RED"
        return best_color

    def cannot_play_and_wait(self) -> None:
        print("No puedo jugar:(, esperando la próxima carta...")
        self.publish_status("no_puedo_jugar")
        self.play_sound(2)

    # ------------------------------------------------------------------
    # Flujo principal de una ronda
    # ------------------------------------------------------------------

    def run(self) -> None:
        if not self.waypoints:
            rospy.logerr("No hay waypoints. Cancelo.")
            return

        if not self.ensure_move_base():
            rospy.logwarn("El nodo seguira esperando, pero no podra navegar hasta que move_base este activo.")

        while not rospy.is_shutdown():
            print("Esperando carta del oponente...")
            self.publish_status("esperando_carta_oponente")
            opponent_card = self.wait_for_card(
                timeout=self.scan_each_card_timeout,
                wait_until_seen=True,
            )
            if opponent_card is None:
                continue

            rospy.loginfo("Carta del oponente: %s conf=%.2f", opponent_card, opponent_card.confidence)
            self.play_sound(4)

            active_color = opponent_card.color

            if opponent_card.rank in ("SKIP", "REVERSE"):
                rospy.loginfo(
                    "El oponente ha jugado %s. No puedo jugar ninguna carta, he perdido el turno :(.",
                    opponent_card.rank,
                )
                self.cannot_play_and_wait()
                continue

            if opponent_card.rank == "WILD":
                active_color = self.ask_active_color(
                    "El oponente ha jugado cambio de color. ¿Qué color ha elegido? [RED/GREEN/BLUE/YELLOW]: "
                )
                rospy.loginfo("Color activo: %s", active_color)

            if opponent_card.rank == "DRAW4":
                active_color = self.ask_active_color(
                    "El oponente ha jugado +4. ¿Qué color ha elegido? [RED/GREEN/BLUE/YELLOW]: "
                )
                rospy.loginfo("Color activo tras +4: %s", active_color)

            scanned_cards = self.scan_all_waypoints()
            chosen = self.choose_card(opponent_card, active_color, scanned_cards)

            if chosen is None:
                self.cannot_play_and_wait()
                continue

            ok, detail = self.go_to_waypoint(chosen.waypoint)
            if not ok:
                rospy.logwarn("He elegido %s pero no puedo volver a %s: %s", chosen.card, chosen.waypoint.name, detail)
                self.cannot_play_and_wait()
                continue

            self.stop_robot()
            print(f"Juego la carta: {chosen.card} en {chosen.waypoint.name}")
            self.publish_status(f"juego_{chosen.card}_en_{chosen.waypoint.name}")

            if chosen.card.rank in SPECIAL_RANKS:
                selected_color = self.choose_color_from_seen_cards(scanned_cards)
                print(f"Elijo el color: {selected_color}")
                self.play_sound(3)
            else:
                self.play_sound(1)

            print("Ronda terminada. Para jugar otra ronda, vuelve a lanzar el nodo.")
            self.publish_status("ronda_terminada")
            return


def main() -> None:
    try:
        orchestrator = UnoOrchestrator()
        orchestrator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Error en uno_orchestrator: %s", exc)
        try:
            rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(Twist())
        except Exception:
            pass


if __name__ == "__main__":
    main()
