# bt_kit API reference

Use BehaviorTree.CPP v4 from Python. `bt_kit.bringup_bt()` returns the C++ `BT`
namespace, including `BehaviorTreeFactory` and `NodeStatus`. The kit adds Python
callbacks and port helpers to the factory API.

In a Pixi project configured with the channels in
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/), install
`pixi add ros-jazzy-bt-kit` and run scripts with `pixi run python your_script.py`.
See the [kit overview](https://awesomebytes.github.io/cppyy_kit/bt_kit/WHY/) for
examples and measurements. Commands using `pixi run -e bt` require this repository
checkout and its Pixi configuration.

## Usage requirements

- Call `bt = bt_kit.bringup_bt()` once; it returns the `BT` namespace.
- A leaf is `def fn(node): ...; return status`. Status is `bt.NodeStatus.SUCCESS`
  (exactly like C++) or the shortcut `bt_kit.SUCCESS`, also `FAILURE`, `RUNNING`.
  Returning `None` means SUCCESS; a `bool` maps True→SUCCESS, False→FAILURE.
- The **XML is the official BT.CPP XML, verbatim.** XML node tags must match the
  names you register.
- A list of port names declares string ports; a dictionary declares typed ports
  (Pattern 2). Read and write them through the `node` argument.
- Keep the `tree` (and the `factory`) referenced while you tick.

---

## Pattern 1: build and tick a tree (tutorial 1)
*Use for:* the common case, synchronous leaves that act and return a status.

```python
import bt_kit
bt = bt_kit.bringup_bt()

def approach(node):
    print("approaching")
    return bt.NodeStatus.SUCCESS

def check_battery(node):
    return bt.NodeStatus.SUCCESS              # a condition is also a leaf

XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <CheckBattery/>
      <ApproachObject/>
    </Sequence>
  </BehaviorTree>
</root>
"""

factory = bt.BehaviorTreeFactory()
factory.registerSimpleCondition("CheckBattery", check_battery)
factory.registerSimpleAction("ApproachObject", approach)
tree = factory.createTreeFromText(XML)        # inline text OR a path to an .xml file
status = tree.tickWhileRunning()              # ticks until root stops RUNNING
assert status == bt_kit.SUCCESS
```
(`registerSimpleAction` / `register_simple_action` and `createTreeFromText` /
`create_tree_from_text` both exist, camelCase mirrors C++, snake_case is there if
you prefer it.)

---

## Pattern 2: ports and the blackboard (tutorial 2)
*Use for:* passing values into a leaf from the XML, or between leaves via a
`{blackboard}` entry.

```python
def say(node):
    print(node.get_input("message"))          # read an input port -> str
    return bt_kit.SUCCESS                      # C++: node.getInput<std::string>("message")

def think(node):
    node.set_output("text", "The answer is 42")  # write an output port
    return bt_kit.SUCCESS

factory.registerSimpleAction("SaySomething", say, ports=["message"])
factory.registerSimpleAction("ThinkWhatToSay", think, ports=["text"])
```
```xml
<Sequence>
  <SaySomething   message="hello world"/>       <!-- literal in -->
  <ThinkWhatToSay text="{the_answer}"/>          <!-- writes blackboard key -->
  <SaySomething   message="{the_answer}"/>        <!-- reads it back -->
</Sequence>
```
`node.get_input(key)` returns a `str` (or `None` if unset); `node.set_output(key,
value)` writes `str(value)`. `node["key"]` / `node["key"] = v` and the camelCase
`getInput`/`setOutput` work too. Ports are bidirectional, the same declared name
serves both reading and writing.

**Typed ports** (tutorial-3 style). Pass a dict to declare types, then read with a
cast (mirrors `getInput<T>`); `set_output` infers the C++ type from the value:
```python
def compute(node):
    total = node.get_input("a", int) + node.get_input("b", float)  # getInput<int>, <double>
    node.set_output("sum", total)          # double inferred
    node.set_output("ok", total > 0)       # bool inferred
    return bt_kit.SUCCESS

factory.registerSimpleAction("Compute", compute,
                             ports={"a": int, "b": float, "sum": float, "ok": bool})
```
Supported casts: `int`, `float` (double), `bool`, `str`, and `[int]`/`[float]`/
`[bool]`/`[str]` for vector ports (e.g. XML `items="1.5;2.5;3.5"` →
`node.get_input("items", [float])` → `[1.5, 2.5, 3.5]`).

---

## Pattern 3: asynchronous and stateful leaves
*Use for:* a long-running action not done in one tick (navigation, waiting,
counting). Use `register_stateful` for actions that return RUNNING, with
a class exposing the C++ StatefulActionNode hooks.

```python
class CountTo:
    def onStart(self, node):                   # called once when the node starts
        self.target = int(node.get_input("count") or 3)
        self.n = 0
        return bt.NodeStatus.RUNNING
    def onRunning(self, node):                 # called every subsequent tick
        self.n += 1
        return bt.NodeStatus.SUCCESS if self.n >= self.target else bt.NodeStatus.RUNNING
    def onHalted(self, node):                  # optional: called if interrupted
        pass

factory.register_stateful("CountTo", CountTo, ports=["count"])
```
(`onStart`/`onRunning`/`onHalted` mirror C++; snake_case `on_start`/... also work.)
Each tree-node instance gets its **own** Python object, so two `<CountTo>` nodes in
one tree keep independent state.

---

## Pattern 4: run a leaf in C++
*Use for:* a hot leaf that must not pay the Python boundary cost (~0.3 µs/tick),
e.g. a tight inner check ticked millions of times. Write it in C++ and JIT it.
(Python leaves reached about 630k ticks/s in the benchmark. Use this pattern if
profiling says so.)

```python
import cppyy
bt = bt_kit.bringup_bt()

cppyy.cppdef(r"""
namespace mykit {
  inline void registerFast(BT::BehaviorTreeFactory& f) {
    f.registerSimpleAction("FastCheck",
      [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
  }
}
""")
factory = bt.BehaviorTreeFactory()
cppyy.gbl.mykit.registerFast(factory)          # C++ registration, bypasses the kit
tree = factory.createTreeFromText(XML)
tree.tickWhileRunning()                         # returns a BT::NodeStatus
```

---

## Pattern 5: logs and errors
*Use for:* watching a tree run, recording a trace, live monitoring, or per-node
tick counts; and for catching malformed-XML / unknown-node errors cleanly.

```python
tree = factory.create_tree_from_text(XML)
bt_kit.add_cout_logger(tree)                 # print transitions to stdout
bt_kit.add_file_logger(tree, "run.btlog")    # record a Groot2-replayable trace
bt_kit.add_groot2_publisher(tree, 1667)      # live monitor over ZMQ (default port)
obs = bt_kit.observe(tree)                    # per-node statistics
tree.tickWhileRunning()
print(obs.counts())    # {node_path: {"transitions": n, "success": n, "failure": n}}

# XML / registration errors raise a readable one-line exception:
try:
    factory.create_tree_from_text('<root BTCPP_format="4">...<Nope/>...</root>')
except bt_kit.BtXmlError as e:
    print(e)   # e.g. "RuntimeError: Error at line 4: -> Node not recognized: Nope"
```
Loggers attach at construction and are pinned on the `tree`; keep the tree alive.

## Pattern 6: compile cache and warmup
*Use for:* any script/node where the **first** tree build must not stall. The first
`registerSimpleAction` etc. would JIT-compile a cppyy call wrapper (~0.4 s; whole
first tree ~0.7 s). bt_kit **compile-caches** this call: registration
routes through a trampoline compiled once into a `.so`, so the first-use register
is ~60 ms and *persistent* (no per-run JIT). You do nothing; it happens at
`bringup_bt()`. The first run on a machine pays a one-time ~2 s `.so` build.

`warmup()` is now a **no-op on the cached path** (nothing to front-load) and stays
useful only as the fallback when no compiler/CPyCppyy toolchain is present (the kit
prints a one-time notice and uses the cppyy JIT path; `bt_kit._CACHED` tells you
which). Force the JIT path with `CPPYY_KIT_NO_CACHE=1`.

```python
import bt_kit

def main():
    bt = bt_kit.bringup_bt()      # cached trampoline adopted here; no warmup needed
    # ... build and tick your real trees; first tick is already fast ...
    # bt_kit.warmup()             # only helps on the JIT fallback path
```
The Cling precompiled header cache can also reduce header parsing on later runs.
The `freeze-bt-run` task is a manual alternative in this repository checkout.
See [cached headers](https://awesomebytes.github.io/cppyy_kit/docs/FREEZE/) and
[compile cache usage](https://awesomebytes.github.io/cppyy_kit/docs/COMMON_PATTERNS/#compile-cache)
for timing conditions and cache requirements.

## Common errors
- **Don't** subclass BT C++ node classes in Python (`class X(BT.StatefulActionNode)`)
 , fails to compile (`final` virtuals). Use `register_stateful` (Pattern 3).
- **Don't** build a `PortsList`/`std::map` in Python, it **segfaults** the
  process. Use `ports=[...]` / `ports={...}`, or do container work in `cppyy.cppdef`.
- Keep the `tree` (and `factory`) referenced while ticking; dropping them can free
  the callbacks and the loggers.
- Multiple `<BehaviorTree>` defs in one XML need the root's
  `main_tree_to_execute="…"` attribute (else you get a clear `BtXmlError`).
- `tree.tickWhileRunning()` returns a `BT::NodeStatus`; it compares equal to the
  `bt_kit.SUCCESS` / `FAILURE` / `RUNNING` ints and to `bt.NodeStatus.*`.
