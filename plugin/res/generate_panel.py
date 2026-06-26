import re
import xml.etree.ElementTree as ET

# Register namespace to preserve it
ET.register_namespace('', 'http://www.w3.org/2000/svg')

# 1. Parse the original SVG to extract the wave paths
orig_path = '/Users/vmaurer/Music/Workshop_VCV_Dev/Workshop_Computer_VCV/res/WorkshopComputer.svg'
tree = ET.parse(orig_path)
root = tree.getroot()

wave_group_str = ""
for el in root.iter():
    # Find the wave group by its ID
    if el.get('id') == 'uuid-53cf9535-c3ae-4b4f-8f24-ffb70e6808b2':
        # Convert the element back to string
        wave_group_str = ET.tostring(el, encoding='utf-8').decode('utf-8')
        break

if not wave_group_str:
    print("Warning: Wave group not found, using empty string")

# Generate the 12 LED sockets
led_sockets_str = ""
for i in range(6):
    y = 322 + i * 7
    # Left LED socket (IN)
    led_sockets_str += f'    <circle cx="20" cy="{y}" r="1.8" style="fill:rgb(30,30,30);stroke:rgb(80,80,80);stroke-width:0.5px;"/>\n'
    # Right LED socket (OUT)
    led_sockets_str += f'    <circle cx="40" cy="{y}" r="1.8" style="fill:rgb(30,30,30);stroke:rgb(80,80,80);stroke-width:0.5px;"/>\n'

# 2. Build the new VCVBridge.svg content
svg_content = f"""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="100%" height="100%" viewBox="0 0 60 380" version="1.1" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" xml:space="preserve" style="fill-rule:evenodd;clip-rule:evenodd;stroke-miterlimit:10;">
    <defs>
        <!-- Port template (centered at 0,0) -->
        <g id="port-template">
            <!-- Dashed outer ring -->
            <circle cx="0" cy="0" r="11.16" style="fill:none;stroke:rgb(193,193,193);stroke-width:1.5px;stroke-dasharray:1.5,1.5;"/>
            <!-- Two horizontal lines -->
            <line x1="-11.05" y1="-1.01" x2="11.16" y2="-1.01" style="stroke:rgb(115,115,115);stroke-width:1px;"/>
            <line x1="-11.05" y1="2.15" x2="10.95" y2="2.15" style="stroke:rgb(124,124,124);stroke-width:1px;"/>
            <!-- Base silver ring -->
            <circle cx="0" cy="0" r="7.92" style="fill:rgb(233,233,234);stroke:rgb(146,146,146);stroke-width:1px;"/>
            <!-- Inner grey core -->
            <circle cx="0" cy="0" r="6.12" style="fill:rgb(137,137,137);stroke:rgb(179,179,179);stroke-width:1px;"/>
        </g>
        <!-- Screw slot template -->
        <g id="screw-template">
            <rect x="-7.5" y="-2" width="15" height="4" rx="2" style="fill:rgb(30,30,30);stroke:rgb(50,50,50);stroke-width:0.5px;"/>
        </g>
    </defs>

    <!-- Background Gray Panel -->
    <rect x="0" y="0" width="60" height="380" style="fill:rgb(61,61,61);"/>

    <!-- Decorative Bottom Wave (scaled horizontally by 0.5 to fit 60px) -->
    <g transform="scale(0.5, 1.0)">
        {wave_group_str}
    </g>

    <!-- Screw Slots -->
    <use href="#screw-template" x="15" y="7.5"/>
    <use href="#screw-template" x="30" y="372.5"/>

    <!-- Text Labels -->
    <!-- Title -->
    <text x="30" y="13" font-family="sans-serif" font-size="7" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">VCV BRIDGE</text>

    <!-- Status/RX/TX LED Labels -->
    <text x="18" y="31" font-family="sans-serif" font-size="4.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">ST</text>
    <text x="30" y="31" font-family="sans-serif" font-size="4.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">RX</text>
    <text x="42" y="31" font-family="sans-serif" font-size="4.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">TX</text>

    <!-- Knob/Switch Output Labels -->
    <text x="15" y="42" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">MAIN</text>
    <text x="45" y="42" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">X</text>
    <text x="15" y="78" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">Y</text>
    <text x="45" y="78" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">SW</text>

    <!-- Inputs Header & Column Indicators -->
    <line x1="5" y1="104" x2="14" y2="104" style="stroke:rgb(212,175,55);stroke-width:1px;"/>
    <text x="30" y="107" font-family="sans-serif" font-size="6" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">INPUTS</text>
    <line x1="46" y1="104" x2="55" y2="104" style="stroke:rgb(212,175,55);stroke-width:1px;"/>

    <text x="15" y="115" font-family="sans-serif" font-size="5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">1</text>
    <text x="45" y="115" font-family="sans-serif" font-size="5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">2</text>

    <!-- Input Row Labels -->
    <text x="30" y="124" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">AUD</text>
    <text x="30" y="160" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">CV</text>
    <text x="30" y="196" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">PLS</text>

    <!-- Outputs Header & Column Indicators (Moved up) -->
    <line x1="5" y1="212" x2="11" y2="212" style="stroke:rgb(212,175,55);stroke-width:1px;"/>
    <text x="30" y="215" font-family="sans-serif" font-size="6" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">OUTPUTS</text>
    <line x1="49" y1="212" x2="55" y2="212" style="stroke:rgb(212,175,55);stroke-width:1px;"/>

    <text x="15" y="223" font-family="sans-serif" font-size="5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">1</text>
    <text x="45" y="223" font-family="sans-serif" font-size="5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">2</text>

    <!-- Output Row Labels (Moved up) -->
    <text x="30" y="232" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">AUD</text>
    <text x="30" y="268" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">CV</text>
    <text x="30" y="304" font-family="sans-serif" font-size="5.5" font-weight="bold" fill="rgb(192,192,192)" text-anchor="middle">PLS</text>

    <!-- LED Level Labels -->
    <text x="20" y="317" font-family="sans-serif" font-size="4.5" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">IN</text>
    <text x="40" y="317" font-family="sans-serif" font-size="4.5" font-weight="bold" fill="rgb(212,175,55)" text-anchor="middle">OUT</text>

    <!-- LED Sockets -->
{led_sockets_str}
    <!-- Port Instances -->
    <!-- Row 1: Knobs Main, X -->
    <use href="#port-template" x="15" y="50"/>
    <use href="#port-template" x="45" y="50"/>

    <!-- Row 2: Knob Y, Switch -->
    <use href="#port-template" x="15" y="86"/>
    <use href="#port-template" x="45" y="86"/>

    <!-- Inputs (Rows 3, 4, 5) -->
    <use href="#port-template" x="15" y="122"/>
    <use href="#port-template" x="45" y="122"/>
    <use href="#port-template" x="15" y="158"/>
    <use href="#port-template" x="45" y="158"/>
    <use href="#port-template" x="15" y="194"/>
    <use href="#port-template" x="45" y="194"/>

    <!-- Outputs (Rows 6, 7, 8 - Moved up one row) -->
    <use href="#port-template" x="15" y="230"/>
    <use href="#port-template" x="45" y="230"/>
    <use href="#port-template" x="15" y="266"/>
    <use href="#port-template" x="45" y="266"/>
    <use href="#port-template" x="15" y="302"/>
    <use href="#port-template" x="45" y="302"/>
</svg>
"""

with open('/Users/vmaurer/Music/Workshop_Computer_VCV_Bridge/plugin/res/VCVBridge.svg', 'w') as f:
    f.write(svg_content)

print("VCVBridge.svg generated successfully.")
