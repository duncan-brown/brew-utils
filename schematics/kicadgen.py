"""Minimal KiCad 9/10 schematic writer.

Writes .kicad_sch files (format 20250114) from Python: stock symbols pulled
out of the installed KiCad libraries, simple home-made box symbols with named
pins, wires, junctions, net labels, global labels, text and no-connects.
Coordinates are millimetres on the 1.27 grid, +y down as in the editor.
"""
import math
import re
import uuid as _uuid

KICAD_SYMS = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols/'
G = 1.27  # grid


def uid():
    return str(_uuid.uuid4())


def snap(v):
    return round(round(v / G) * G, 4)


def f(v):
    """Format a number the way KiCad does."""
    s = f'{v:.4f}'.rstrip('0').rstrip('.')
    return s if s not in ('', '-0') else '0'


# ---------------------------------------------------------------- S-expressions
def parse(text):
    """Parse an S-expression string into nested lists (atoms as str)."""
    tokens = re.findall(r'"(?:[^"\\]|\\.)*"|\(|\)|[^\s()]+', text)
    stack = [[]]
    for t in tokens:
        if t == '(':
            stack.append([])
        elif t == ')':
            top = stack.pop()
            stack[-1].append(top)
        else:
            stack[-1].append(t)
    return stack[0]


def dump(node, depth=0):
    if isinstance(node, str):
        return node
    parts = [dump(n, depth + 1) for n in node]
    flat = '(' + ' '.join(parts) + ')'
    if len(flat) < 100 and not any(isinstance(n, list) for n in node):
        return flat
    ind = '\t' * (depth + 1)
    out = '(' + (parts[0] if parts else '')
    for p in parts[1:]:
        if p.startswith('('):
            out += '\n' + ind + p
        else:
            out += ' ' + p
    return out + '\n' + '\t' * depth + ')'


def q(s):
    return '"' + str(s).replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'


def find(node, key):
    return [n for n in node if isinstance(n, list) and n and n[0] == key]


# --------------------------------------------------------------- symbol sources
class LibSymbol:
    """A symbol definition destined for lib_symbols, plus its pin geometry."""

    def __init__(self, lib_id, sexpr, pins, size=None):
        self.lib_id = lib_id          # e.g. "Device:R"
        self.sexpr = sexpr            # parsed list, with name already set to lib_id
        self.pins = pins              # {number: (x, y, angle, name)} in lib coords (+y up)
        if size is None and pins:
            xs = [p[0] for p in pins.values()]; ys = [p[1] for p in pins.values()]
            size = (max(xs) - min(xs) or 2.54, max(ys) - min(ys) or 2.54)
        self.size = size or (2.54, 2.54)


_stock_cache = {}


def stock(lib, name):
    """Load a stock symbol like stock('Device', 'R'), including inherited parents."""
    key = (lib, name)
    if key in _stock_cache:
        return _stock_cache[key]
    if lib not in _stock_cache:
        with open(KICAD_SYMS + lib + '.kicad_sym') as fh:
            _stock_cache[lib] = parse(fh.read())[0]
    root = _stock_cache[lib]
    syms = {s[1].strip('"'): s for s in find(root, 'symbol')}
    if name not in syms:
        raise KeyError(f'{lib}:{name} not in library')
    node = syms[name]
    # flatten "extends" (derived symbols) by copying the parent's drawing units
    ext = find(node, 'extends')
    if ext:
        parent = syms[ext[0][1].strip('"')]
        merged = [n for n in parent if not (isinstance(n, list) and n and n[0] == 'property')]
        merged = [n for n in merged if not (isinstance(n, list) and n and n[0] == 'extends')]
        props = find(node, 'property')
        merged = merged[:1] + props + [n for n in merged[1:] if not (isinstance(n, list) and n[0] == 'property')]
        node = merged
        node = [n for n in node]
        # rename sub-units to the derived name
        pname = parent[1].strip('"')
        for i, n in enumerate(node):
            if isinstance(n, list) and n and n[0] == 'symbol':
                node[i] = list(n)
                node[i][1] = q(n[1].strip('"').replace(pname, name, 1))
    node = list(node)
    node[1] = q(f'{lib}:{name}')
    pins = {}
    for unit in find(node, 'symbol'):
        for p in find(unit, 'pin'):
            at = find(p, 'at')[0]
            num = find(p, 'number')[0][1].strip('"')
            nm = find(p, 'name')[0][1].strip('"')
            pins[num] = (float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0, nm)
    ls = LibSymbol(f'{lib}:{name}', node, pins)
    _stock_cache[key] = ls
    return ls


def box_symbol(name, left=(), right=(), top=(), bottom=(), width=None, ref='U', pinlen=2.54,
               spacing=2.54, hide_pin_numbers=True):
    """A rectangular symbol with named pins.  left/right/top/bottom are lists of
    pin names (top-to-bottom / left-to-right); pin numbers are assigned in order."""
    n_side = max(len(left), len(right), 1)
    n_tb = max(len(top), len(bottom), 1)
    h = max((n_side + 1) * spacing, 2 * spacing)
    if width is None:
        lmax = max([len(p) for p in left] + [0]); rmax = max([len(p) for p in right] + [0])
        width = max(snap(lmax * 1.35 + rmax * 1.35 + 7.62), (n_tb + 1) * spacing, 12.7)
    import math as _m
    w = round(_m.ceil(width / 2.54) * 2.54, 4)   # even grid so side pins sit on the 1.27 grid
    h = round(_m.ceil(h / 2.54) * 2.54, 4)
    pins = {}
    items = []
    num = 1

    def ypos(i, n):
        # centre the n pins vertically on the grid
        return snap((n - 1) * spacing / 2 - i * spacing)

    def xpos(i, n):
        return snap(-(n - 1) * spacing / 2 + i * spacing)

    for i, p in enumerate(left):
        pins[str(num)] = (-w / 2 - pinlen, ypos(i, len(left)), 0.0, p); num += 1
    for i, p in enumerate(right):
        pins[str(num)] = (w / 2 + pinlen, ypos(i, len(right)), 180.0, p); num += 1
    for i, p in enumerate(top):
        pins[str(num)] = (xpos(i, len(top)), h / 2 + pinlen, 270.0, p); num += 1
    for i, p in enumerate(bottom):
        pins[str(num)] = (xpos(i, len(bottom)), -h / 2 - pinlen, 90.0, p); num += 1

    pin_sx = []
    for number, (x, y, ang, nm) in pins.items():
        pin_sx.append(parse(
            f'(pin passive line (at {f(x)} {f(y)} {f(ang)}) (length {f(pinlen)}) '
            f'(name {q(nm)} (effects (font (size 1.016 1.016)))) '
            f'(number {q(number)} (effects (font (size 1.016 1.016)))))')[0])
    rect = parse(f'(rectangle (start {f(-w/2)} {f(h/2)}) (end {f(w/2)} {f(-h/2)}) '
                 f'(stroke (width 0.254) (type default)) (fill (type background)))')[0]
    lib_id = f'brew:{name}'
    sx = parse(
        f'(symbol {q(lib_id)} (pin_names (offset 0.762)) (exclude_from_sim no) (in_bom yes) (on_board yes) '
        f'(property "Reference" {q(ref)} (at 0 {f(h/2 + 1.27)} 0) (effects (font (size 1.27 1.27)))) '
        f'(property "Value" {q(name)} (at 0 {f(-h/2 - 1.27)} 0) (effects (font (size 1.27 1.27)))) '
        f'(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes))) '
        f'(property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes))) '
        f'(property "Description" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes))) '
        f'(symbol {q(name + "_0_1")}) (symbol {q(name + "_1_1")}))')[0]
    if hide_pin_numbers:
        sx.insert(2, parse('(pin_numbers (hide yes))')[0])
    find(sx, 'symbol')[0].append(rect)
    find(sx, 'symbol')[1].extend(pin_sx)
    return LibSymbol(lib_id, sx, pins, size=(w, h))


# ------------------------------------------------------------------- schematic
class Placed:
    def __init__(self, lib, x, y, rot, mirror, ref, value):
        self.lib, self.x, self.y, self.rot, self.mirror, self.ref, self.value = lib, x, y, rot, mirror, ref, value
        self.uuid = uid()

    def pin(self, key):
        """Sheet coordinates of a pin, by number or by name."""
        if key not in self.lib.pins:
            for num, (_, _, _, nm) in self.lib.pins.items():
                if nm == key:
                    key = num
                    break
            else:
                raise KeyError(f'{self.lib.lib_id} has no pin {key}')
        px, py, _, _ = self.lib.pins[key]
        if self.mirror == 'y':
            px = -px
        if self.mirror == 'x':
            py = -py
        th = math.radians(self.rot)
        rx = px * math.cos(th) - py * math.sin(th)
        ry = px * math.sin(th) + py * math.cos(th)
        return (round(self.x + rx, 4), round(self.y - ry, 4))

    def __getitem__(self, key):
        return self.pin(key)


class Schematic:
    def __init__(self, title, paper='A3', rev='1', company='brew-utils'):
        self.title, self.paper, self.rev, self.company = title, paper, rev, company
        self.uuid = uid()
        self.libs = {}
        self.items = []
        self.refs = {}

    def _ref(self, prefix):
        self.refs[prefix] = self.refs.get(prefix, 0) + 1
        return f'{prefix}{self.refs[prefix]}'

    def place(self, lib, x, y, rot=0, ref=None, value=None, mirror=None, hide_value=False,
              value_at=None, ref_at=None, hide_ref=False):
        self.libs[lib.lib_id] = lib
        if ref is None:
            prefix = find(lib.sexpr, 'property')
            prefix = [p for p in prefix if p[1] == '"Reference"'][0][2].strip('"')
            ref = self._ref(prefix)
        p = Placed(lib, x, y, rot, mirror, ref, value or '')
        p.hide_value, p.value_at, p.ref_at, p.hide_ref = hide_value, value_at, ref_at, hide_ref
        self.items.append(('symbol', p))
        return p

    def wire(self, *pts):
        pts = [pts[0]] + list(pts[1:])
        for a, b in zip(pts, pts[1:]):
            self.items.append(('wire', (a, b)))

    def route(self, a, b, via='h'):
        """Orthogonal two-segment wire, horizontal-first ('h') or vertical-first ('v')."""
        if via == 'h':
            m = (b[0], a[1])
        else:
            m = (a[0], b[1])
        if m == a or m == b:
            self.wire(a, b)
        else:
            self.wire(a, m, b)
        return b

    def junction(self, pt):
        self.items.append(('junction', pt))

    def label(self, pt, text, rot=0, just='left bottom'):
        self.items.append(('label', (pt, text, rot, just)))

    def glabel(self, pt, text, rot=0, shape='bidirectional'):
        self.items.append(('glabel', (pt, text, rot, shape)))

    def text(self, pt, text, size=1.27, rot=0, just='left top'):
        self.items.append(('text', (pt, text, size, rot, just)))

    def nc(self, pt):
        self.items.append(('nc', pt))

    def write(self, path):
        out = ['(kicad_sch', '\t(version 20250114)', '\t(generator "brew-utils")', '\t(generator_version "1")',
               f'\t(uuid {q(self.uuid)})', f'\t(paper {q(self.paper)})',
               '\t(title_block', f'\t\t(title {q(self.title)})', f'\t\t(rev {q(self.rev)})',
               f'\t\t(company {q(self.company)})', '\t)', '\t(lib_symbols']
        for lib in self.libs.values():
            out.append('\t\t' + dump(lib.sexpr, 2).replace('\n', '\n\t\t'))
        out.append('\t)')
        for kind, it in self.items:
            if kind == 'wire':
                (a, b) = it
                out.append(f'\t(wire (pts (xy {f(a[0])} {f(a[1])}) (xy {f(b[0])} {f(b[1])})) '
                           f'(stroke (width 0) (type default)) (uuid {q(uid())}))')
            elif kind == 'junction':
                out.append(f'\t(junction (at {f(it[0])} {f(it[1])}) (diameter 0) (color 0 0 0 0) (uuid {q(uid())}))')
            elif kind == 'nc':
                out.append(f'\t(no_connect (at {f(it[0])} {f(it[1])}) (uuid {q(uid())}))')
            elif kind == 'label':
                (pt, text, rot, just) = it
                out.append(f'\t(label {q(text)} (at {f(pt[0])} {f(pt[1])} {f(rot)}) '
                           f'(effects (font (size 1.27 1.27)) (justify {just})) (uuid {q(uid())}))')
            elif kind == 'glabel':
                (pt, text, rot, shape) = it
                just = 'left' if rot in (0, 90) else 'right'
                out.append(f'\t(global_label {q(text)} (shape {shape}) (at {f(pt[0])} {f(pt[1])} {f(rot)}) '
                           f'(fields_autoplaced yes) (effects (font (size 1.27 1.27)) (justify {just})) (uuid {q(uid())}) '
                           f'(property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes))))')
            elif kind == 'text':
                (pt, text, size, rot, just) = it
                out.append(f'\t(text {q(text)} (exclude_from_sim no) (at {f(pt[0])} {f(pt[1])} {f(rot)}) '
                           f'(effects (font (size {f(size)} {f(size)})) (justify {just})) (uuid {q(uid())}))')
            elif kind == 'symbol':
                p = it
                mir = f' (mirror {p.mirror})' if p.mirror else ''
                pins = ' '.join(f'(pin {q(n)} (uuid {q(uid())}))' for n in p.lib.pins)
                # put Reference/Value just above/below the body by default
                w, h = p.lib.size
                if p.rot in (90, 270):
                    w, h = h, w
                ra = p.ref_at or (p.x, p.y - h / 2 - 1.9)
                va = p.value_at or (p.x, p.y + h / 2 + 1.9)
                hv = ' (hide yes)' if p.hide_value else ''
                hr = ' (hide yes)' if p.hide_ref else ''
                out.append(
                    f'\t(symbol (lib_id {q(p.lib.lib_id)}) (at {f(p.x)} {f(p.y)} {f(p.rot)}){mir} (unit 1) '
                    f'(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid {q(p.uuid)}) '
                    f'(property "Reference" {q(p.ref)} (at {f(ra[0])} {f(ra[1])} 0) (effects (font (size 1.27 1.27)){hr})) '
                    f'(property "Value" {q(p.value)} (at {f(va[0])} {f(va[1])} 0) (effects (font (size 1.27 1.27)){hv})) '
                    f'(property "Footprint" "" (at {f(p.x)} {f(p.y)} 0) (effects (font (size 1.27 1.27)) (hide yes))) '
                    f'(property "Datasheet" "" (at {f(p.x)} {f(p.y)} 0) (effects (font (size 1.27 1.27)) (hide yes))) '
                    f'{pins} (instances (project "brew-utils" (path {q("/" + self.uuid)} (reference {q(p.ref)}) (unit 1)))))')
        out.append('\t(sheet_instances (path "/" (page "1")))')
        out.append(')')
        with open(path, 'w') as fh:
            fh.write('\n'.join(out) + '\n')
        return path
