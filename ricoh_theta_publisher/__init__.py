"""ROS 2 nodes for interacting with RICOH Theta cameras."""

from importlib import import_module
from typing import Any, TYPE_CHECKING

from . import capture

__all__ = ["string_publisher", "capture"]


if TYPE_CHECKING:  # pragma: no cover
    from . import string_publisher


def __getattr__(name: str) -> Any:
    if name in __all__:
        module = import_module(f".{name}", __name__)
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
