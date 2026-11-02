from pathlib import Path

import pytest

import frontierfly

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def env():
    e = frontierfly.make("hard")
    yield e
    e.close()


@pytest.fixture
def repo() -> Path:
    return REPO
