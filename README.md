# TestFit Clone — Parking Solver · Site Solver · Site Intelligence

A browser-based real-estate feasibility tool that re-creates the core
functionality of three TestFit products:

- **Parking Solver** — auto-generate an optimised parking layout on any parcel.
- **Site Solver** — auto-generate building massing, unit mix, and a financial
  pro-forma, with parking wrapped around it.
- **Site Intelligence** — buildable-envelope (setback) analysis on a real
  satellite map, plus a zoning pass/fail compliance scorecard.

No build step, no framework. Open `index.html` (the real-world map needs
internet for tiles; everything else works offline).

## Run

- Double-click `index.html`, **or**
- Serve the folder: `python3 -m http.server 8801` → http://localhost:8801

## Files

| File | Purpose |
|------|---------|
| `index.html` | Layout, styles, all controls (parking + site panels) |
| `solver.js`  | Pure geometry engine: parking packing **and** site massing/zoning |
| `app.js`     | Canvas + Leaflet rendering, tools, modes, metrics, export, 3D |

## Modes (top-left toggle)

### 🅿️ Parking
- Auto-pack double-loaded modules; orientation aligned to parcel edges.
- 90° / 60° / 45°; adjustable stall/aisle dims; perimeter setback.
- Avoids buildings & obstacles; ADA (2010 table) / EV / compact mix.
- Live: total stalls, ratio /1,000 SF, SF/stall, acreage, coverage.

### 🏢 Site (Site Solver + Site Intelligence)
- **Buildable envelope** = parcel inset by front/side/rear setbacks
  (Sutherland–Hodgman half-plane clip; front edge inferred from the entrance).
- **Massing**: footprint capped by max coverage; floors limited by *both*
  height and FAR; GFA / NRSF / FAR / coverage computed.
- **Unit mix** (studio/1/2/3-bed) → unit count, density-capped at DU/acre.
- **Parking** auto-packed around the building → provided vs required.
- **Pro-forma**: land + hard + soft cost, NOI, yield-on-cost.
- **Compliance scorecard**: FAR / height / coverage / parking / density,
  each PASS/FAIL — the Site-Intelligence "pass/fail scheme scoring".
- Building types: multifamily, mixed-use, office, retail, industrial.

## Real-world map base (free, no API key)

Toggle **地圖**. Uses Leaflet + Esri World Imagery (satellite) / OpenStreetMap
(street) tiles and OSM Nominatim geocoding — all free, no key, no billing.
Search an address, then trace the parcel directly on the imagery; the overlay
is locked to the map (it pans/zooms with the imagery) and parcel dimensions are
read from the real-world scale. *Google Maps tiles are intentionally not used —
they require your own API key + a billing account.*

## Units & regions

- **ft / m toggle** (toolbar): all dimensions, areas (SF↔m², ac↔ha) and labels
  convert live. Geometry is stored canonically in feet; only display converts.
- **Region presets** 🇺🇸🇹🇼🇯🇵🇪🇺: each sets the unit system + typical stall/aisle
  dimensions + a starting zoning profile (FAR / height / coverage / setbacks /
  parking ratio). These are EDITABLE reference defaults, not binding local code.

## Editing

- **Entrances**: place with the 出入口 tool; in 選取 mode drag to move,
  double-click to cycle 進出 / 只進 / 只出, right-click or Delete to remove.
  Moving/placing/deleting an entrance **re-solves** and the drive lane
  (circulation) carved from each entrance into the site updates accordingly.
- **Parking stalls**: double-click to cycle type, right-click/Delete to remove.
- **Parcel vertices**: drag in 選取 mode.

Note: the 3D massing view and the real-world map are mutually exclusive
(entering one exits the other).

## 2D plan / 3D isometric massing
Both modes render in a depth-sorted isometric view with adjustable height
(parking-mode building height, or the Site-Solver generated floors).

## Export
PNG, CSV (parking list, or full site feasibility table), DXF (layered CAD
incl. buildable envelope + massing), JSON (full project). Schemes save/load via
localStorage and remember the mode.

## Honest scope note
TestFit's real zoning / parcel / flood / soil databases are licensed
proprietary feeds. This clone implements the full feature & compliance *engine*
with manual zoning input + free maps — it does not pull live zoning data.

Units are feet throughout.
