import json
import os

class RobotConfig:
    """Clase para cargar y guardar configuración mecánica del robot."""

    def __init__(self, config_path="robot_config.json"):
        self.config_path = config_path
        self.data = self.load_config()

    def load_config(self):
        """Carga la configuración desde el archivo JSON."""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"No se encontró {self.config_path}")
        with open(self.config_path, "r") as file:
            return json.load(file)

    def save_config(self):
        """Guarda la configuración actual."""
        with open(self.config_path, "w") as file:
            json.dump(self.data, file, indent=4)

    def get_axis(self, axis_name):
        """Devuelve los parámetros de un eje específico."""
        return self.data["axes"].get(axis_name)


class MotionModel:
    """Realiza conversiones entre pasos, mm y grados, y controla límites."""

    def __init__(self, config: RobotConfig):
        self.config = config

    def steps_to_units(self, axis, steps):
        """Convierte pasos → grados o mm según el tipo de eje."""
        axis_cfg = self.config.get_axis(axis)
        scale = axis_cfg["steps_per_unit"]
        return steps / scale

    def units_to_steps(self, axis, value):
        """Convierte grados o mm → pasos según el tipo de eje."""
        axis_cfg = self.config.get_axis(axis)
        scale = axis_cfg["steps_per_unit"]
        return int(value * scale)

    def within_limits(self, axis, value):
        """Verifica si el valor solicitado está dentro del rango permitido."""
        axis_cfg = self.config.get_axis(axis)
        min_val = axis_cfg["limits"]["min"]
        max_val = axis_cfg["limits"]["max"]
        return min_val <= value <= max_val

    def clip_to_limits(self, axis, value):
        """Recorta un valor para que no supere los límites."""
        axis_cfg = self.config.get_axis(axis)
        min_val = axis_cfg["limits"]["min"]
        max_val = axis_cfg["limits"]["max"]
        return max(min(value, max_val), min_val)