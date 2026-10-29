"""The models must read every robot number from the config (issue #22).

Allowed literals: small integers used in formulas, 0.5, tolerances, and
UPPER_CASE module constants (physical constants and node defaults, which are
named and reviewed).
"""

import ast
import os

import pytest

PACKAGE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'lawn_mower_robot')
ALLOWED = {0, 1, 2, 3, 12, 0.5}
TOLERANCE = 1e-6


def modules():
    return sorted(f for f in os.listdir(PACKAGE) if f.endswith('.py'))


def named_constant_nodes(tree):
    nodes = set()
    for stmt in tree.body:
        if isinstance(stmt, ast.Assign) and all(
                isinstance(t, ast.Name) and t.id.isupper() for t in stmt.targets):
            nodes.update(id(n) for n in ast.walk(stmt))
    return nodes


@pytest.mark.parametrize('module', modules())
def test_no_literal_robot_numbers(module):
    with open(os.path.join(PACKAGE, module), encoding='utf-8') as f:
        tree = ast.parse(f.read())
    skip = named_constant_nodes(tree)
    found = [
        f'{module}:{node.lineno}: {node.value!r}'
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and id(node) not in skip
        and isinstance(node.value, (int, float)) and not isinstance(node.value, bool)
        and abs(node.value) not in ALLOWED and abs(node.value) > TOLERANCE
    ]
    assert not found, 'literal numbers; put robot values in the config:\n' + '\n'.join(found)
