"""Add matching cosmetic visuals to upstream TB3 SDF and URDF descriptions.

The SDF physics, joints, plugins and sensors are unchanged. URDF adds fixed
cosmetic links by color because RViz applies a common material within a link.
The shell stays below the laser scan plane. Geometry is generated locally.
"""
import math
import xml.etree.ElementTree as ET

import xacro


def write_dome(path):
    """Write a unit upper-hemisphere STL with a closed bottom."""
    triangles = []
    count, bands = 40, 12

    def point(band, segment):
        theta = math.pi * band / (2 * bands)
        phi = 2 * math.pi * segment / count
        return (math.sin(theta) * math.cos(phi),
                math.sin(theta) * math.sin(phi), math.cos(theta))

    for band in range(bands):
        for segment in range(count):
            a, b = point(band, segment), point(band + 1, segment)
            c, d = point(band + 1, segment + 1), point(band, segment + 1)
            triangles.append((a, b, c))
            if band:
                triangles.append((a, c, d))
    for segment in range(count):
        triangles.append(((0, 0, 0), point(bands, segment + 1), point(bands, segment)))
    lines = ['solid turtle_shell']
    for a, b, c in triangles:
        u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        length = math.sqrt(sum(t*t for t in n))
        n = [t/length for t in n]
        lines += ['facet normal ' + ' '.join(map(str, n)), 'outer loop']
        lines += ['vertex ' + ' '.join(map(str, p)) for p in (a, b, c)]
        lines += ['endloop', 'endfacet']
    path.write_text('\n'.join(lines + ['endsolid turtle_shell']))


def build_descriptions(tb3, work):
    sdf = ET.fromstring(xacro.process_file(str(tb3 / 'urdf/gz_waffle.sdf.xacro')).toxml())
    urdf = ET.fromstring((tb3 / 'urdf/turtlebot3_waffle.urdf').read_text())
    sdf_base = sdf.find("model/link[@name='base_link']")
    urdf_base = urdf.find("link[@name='base_link']")
    green, lime, gold = '0.12 0.40 0.19 1', '0.45 0.80 0.25 1', '0.85 0.72 0.20 1'
    dome = work / 'turtle_dome.stl'
    write_dome(dome)

    # Color the existing chassis consistently in both Gazebo and RViz.
    for visual in sdf_base.findall('visual'):
        material = visual.find('material')
        if material is not None:
            visual.remove(material)
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = green
        ET.SubElement(material, 'diffuse').text = green
    for visual in urdf_base.findall('visual'):
        material = visual.find('material')
        if material is not None:
            visual.remove(material)
        ET.SubElement(ET.SubElement(visual, 'material', name='turtle_chassis'), 'color', rgba=green)

    cosmetic_links = {green: urdf_base}

    def visual(name, xyz, color, radius=None, scale=None):
        sv = ET.SubElement(sdf_base, 'visual', name=name)
        ET.SubElement(sv, 'pose').text = f'{xyz} 0 0 0'
        sg = ET.SubElement(sv, 'geometry')
        sm = ET.SubElement(sv, 'material')
        ET.SubElement(sm, 'ambient').text = color
        ET.SubElement(sm, 'diffuse').text = color
        if color not in cosmetic_links:
            link_name = f'turtle_visual_{len(cosmetic_links)}'
            cosmetic_links[color] = ET.SubElement(urdf, 'link', name=link_name)
            joint = ET.SubElement(urdf, 'joint', name=f'{link_name}_joint', type='fixed')
            ET.SubElement(joint, 'parent', link='base_link')
            ET.SubElement(joint, 'child', link=link_name)
            ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        uv = ET.SubElement(cosmetic_links[color], 'visual', name=name)
        ET.SubElement(uv, 'origin', xyz=xyz, rpy='0 0 0')
        ug = ET.SubElement(uv, 'geometry')
        ET.SubElement(ET.SubElement(uv, 'material', name=name), 'color', rgba=color)
        if radius is not None:
            ET.SubElement(ET.SubElement(sg, 'sphere'), 'radius').text = str(radius)
            ET.SubElement(ug, 'sphere', radius=str(radius))
        else:
            mesh = ET.SubElement(sg, 'mesh')
            ET.SubElement(mesh, 'uri').text = dome.as_uri()
            ET.SubElement(mesh, 'scale').text = scale
            ET.SubElement(ug, 'mesh', filename=dome.as_uri(), scale=scale)

    visual('shell_rim', '-0.064 0 0.063', gold, scale='0.137 0.130 0.009')
    visual('shell', '-0.064 0 0.070', green, scale='0.130 0.123 0.050')
    # Raised shell plates follow the dome, producing a segmented turtle shell.
    for i, (dx, dy) in enumerate([(0, 0), (-.065, 0), (.065, 0),
                                  (-.032, -.065), (.032, -.065),
                                  (-.032, .065), (.032, .065)]):
        z = .070 + .05 * math.sqrt(1 - (dx/.13)**2 - (dy/.123)**2) - .004
        visual(f'shell_plate_{i}', f'{-.064+dx} {dy} {z}', lime, scale='.033 .029 .009')
    visual('turtle_head', '0.090 0 0.062', lime, radius=.034)
    for side in [-1, 1]:
        visual(f'eye_{side}', f'0.113 {side*.019} 0.084', '0.96 0.96 0.85 1', radius=.009)
        visual(f'pupil_{side}', f'0.120 {side*.021} 0.087', '0.02 0.03 0.02 1', radius=.0045)
        visual(f'front_flipper_{side}', f'0.025 {side*.120} 0.045', lime, radius=.023)
        visual(f'rear_flipper_{side}', f'-0.160 {side*.110} 0.040', lime, radius=.023)
    visual('tail', '-0.194 0 0.035', lime, radius=.022)
    sdf_path = work / 'turtle_waffle.sdf'
    ET.indent(sdf)
    ET.ElementTree(sdf).write(sdf_path, encoding='unicode', xml_declaration=True)
    return ET.tostring(urdf, encoding='unicode'), sdf_path
