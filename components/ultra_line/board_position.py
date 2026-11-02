"""A board's position on its line in YAML: counted from 1, like num_boards (board 1 = the first board after the
controller). Inside the code (C++, the wire) a board is its slot, 0-based: board_slot() is the one conversion."""

import esphome.config_validation as cv

FIRST_BOARD = "boards count from 1: the first board after the controller is board 1"


def board_position(value):
    """board: 1..64 (a line's own num_boards bounds it in its final validation)"""
    value = cv.int_(value)
    if value < 1:
        raise cv.Invalid(f"board {value}: {FIRST_BOARD}")
    if value > 64:
        raise cv.Invalid(f"board {value}: a line has at most 64 boards")
    return value


def board_slot(position):
    """the 0-based slot of a board position (YAML board: 1 -> slot 0)"""
    return position - 1


def out_of_range(position, hub_id, num_boards):
    """the text for a board position beyond its line's num_boards"""
    return (f"board {position} out of range, '{hub_id}' has num_boards: {num_boards} "
            f"(boards count from 1: board 1 .. {num_boards})")


def positions_text(slots):
    """slots as the board positions a user reads: 1, 2, 3"""
    return ", ".join(str(slot_ + 1) for slot_ in slots)
