"""
tests/test_import_consistency.py

Catches the exact bug class that hit this repo twice: a file imports a
name from another amr_fleet_manager module that doesn't actually exist
there (e.g. `from amr_fleet_manager.robot_common import sign_payload`
when robot_common.py never defines sign_payload).

Deliberately does NOT execute any file -- most node files import rclpy
at module level, which isn't installed here, and a literal `import`
would fail on that environment gap rather than reveal anything about
real bugs. Instead this parses each file's AST and checks imported
names against what the target module actually defines at module level,
which needs nothing but the standard library.

This would have caught robot_common.py's missing sign_payload/
verify_payload immediately, on any machine, with no ROS install.

Run: pytest tests/test_import_consistency.py -v
"""

import ast
import os
import sys

PACKAGE_DIR = os.path.join(os.path.dirname(__file__), '..', 'amr_fleet_manager', 'amr_fleet_manager')


def _module_files():
    """All .py files directly in the amr_fleet_manager package dir."""
    files = {}
    for fname in os.listdir(PACKAGE_DIR):
        if fname.endswith('.py'):
            mod_name = fname[:-3]
            files[mod_name] = os.path.join(PACKAGE_DIR, fname)
    return files


def _top_level_names(filepath):
    """Every name a module defines at module level: function defs, class
    defs, and names assigned to (including tuple/multiple assignment and
    augmented targets like `MUTEX_CLEAR = "CLEAR"`)."""
    with open(filepath, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=filepath)

    names = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    names.add(target.id)
                elif isinstance(target, (ast.Tuple, ast.List)):
                    for elt in target.elts:
                        if isinstance(elt, ast.Name):
                            names.add(elt.id)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                names.add((alias.asname or alias.name).split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                names.add(alias.asname or alias.name)
    return names


def _cross_module_imports(filepath):
    """Every `from amr_fleet_manager.X import name1, name2` in this file,
    as (target_module, [names], line_number). Ignores
    `from amr_fleet_manager import X` (module-level import, not a name
    import -- checked separately by _referenced_modules)."""
    with open(filepath, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=filepath)

    results = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            if node.module.startswith('amr_fleet_manager.'):
                target = node.module.split('.', 1)[1]
                names = [alias.name for alias in node.names]
                results.append((target, names, node.lineno))
    return results


def _referenced_sibling_modules(filepath):
    """Every `from amr_fleet_manager import X` (module import, not a
    name import) -- checked as 'does X.py exist', not 'does X define
    these names'."""
    with open(filepath, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=filepath)

    modules = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == 'amr_fleet_manager':
            for alias in node.names:
                modules.append((alias.name, node.lineno))
    return modules


def test_every_cross_module_name_import_resolves():
    all_modules = _module_files()
    problems = []

    for source_name, source_path in all_modules.items():
        for target_name, imported_names, lineno in _cross_module_imports(source_path):
            if target_name not in all_modules:
                problems.append(
                    f"{source_name}.py:{lineno} imports from '{target_name}', "
                    f"but no amr_fleet_manager/{target_name}.py exists"
                )
                continue

            target_names = _top_level_names(all_modules[target_name])
            for name in imported_names:
                if name not in target_names:
                    problems.append(
                        f"{source_name}.py:{lineno} imports '{name}' from "
                        f"'{target_name}', but {target_name}.py does not define it "
                        f"at module level"
                    )

    assert not problems, "Cross-module import mismatches found:\n" + "\n".join(problems)


def test_every_referenced_sibling_module_exists():
    all_modules = _module_files()
    problems = []

    for source_name, source_path in all_modules.items():
        for target_name, lineno in _referenced_sibling_modules(source_path):
            if target_name not in all_modules:
                problems.append(
                    f"{source_name}.py:{lineno} does 'from amr_fleet_manager import "
                    f"{target_name}', but no amr_fleet_manager/{target_name}.py exists"
                )

    assert not problems, "Missing sibling modules:\n" + "\n".join(problems)


def test_package_dir_is_actually_found():
    """Sanity check on the test's own setup -- if this fails, the two
    tests above are silently checking nothing, which is worse than not
    running them at all."""
    assert os.path.isdir(PACKAGE_DIR), f"expected package dir at {PACKAGE_DIR}"
    assert len(_module_files()) > 0, "found the package dir but zero .py files in it"
