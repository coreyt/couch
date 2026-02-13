"""Pytest configuration for SOFA spike tests."""
import os
import sys
import pytest


def pytest_configure(config):
    """Set up SOFA environment before any tests run."""
    sofa_root = os.environ.get(
        "SOFA_ROOT",
        os.path.expanduser("~/sofa/SOFA_v24.06.00_Linux"),
    )
    sp3_path = os.path.join(
        sofa_root, "plugins", "SofaPython3", "lib", "python3", "site-packages"
    )
    if sp3_path not in sys.path:
        sys.path.insert(0, sp3_path)

    ld = os.environ.get("LD_LIBRARY_PATH", "")
    sofa_lib = os.path.join(sofa_root, "lib")
    if sofa_lib not in ld:
        os.environ["LD_LIBRARY_PATH"] = f"{sofa_lib}:{ld}"
    os.environ["SOFA_ROOT"] = sofa_root


@pytest.fixture
def sofa_root_node():
    """Create a fresh SOFA root node for each test."""
    import Sofa.Core

    root = Sofa.Core.Node("root")
    yield root
    # SOFA cleanup happens automatically when root goes out of scope
