# persistent_var.py

class PersistentVariable:
    def __init__(self, initial_value=None):
        self._value = initial_value

    def get(self):
        return self._value

    def set(self, value):
        self._value = value

    def reset(self, value=None):
        self._value = value