"""Generate six independent Open CASCADE solids and a STEP simulation scene."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

from rgbd_geometry.cad_simulation import SCENE_DEFAULTS


def generate(directory):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    from OCP.TopoDS import TopoDS
    from OCP.gp import gp_Pnt, gp_Vec

    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    names = ("cube", "cuboid", "triangular_pyramid", "triangular_prism", "cone", "cylinder")
    targets = [directory / f"{name}.step" for name in names] + [directory / "scene.json"]
    if any(path.exists() for path in targets):
        raise FileExistsError("示例目标文件已存在；请换一个输出目录，避免覆盖已有模型")

    def face(points):
        polygon = BRepBuilderAPI_MakePolygon()
        for point in points:
            polygon.Add(gp_Pnt(*point))
        polygon.Close()
        return BRepBuilderAPI_MakeFace(polygon.Wire()).Face()

    points = [(-50., -47.5, 0.), (50., -47.5, 0.), (0., 47.5, 0.), (0., -47.5/3, 110.)]
    sewing = BRepBuilderAPI_Sewing()
    for indices in ((0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)):
        sewing.Add(face([points[i] for i in indices]))
    sewing.Perform()
    pyramid = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sewing.SewedShape())).Solid()
    prism_face = face([(-47.5, -37.5, 0.), (47.5, -37.5, 0.), (0., 37.5, 0.)])
    solids = [BRepPrimAPI_MakeBox(75., 75., 75.).Shape(), BRepPrimAPI_MakeBox(115., 65., 60.).Shape(),
              pyramid, BRepPrimAPI_MakePrism(prism_face, gp_Vec(0., 0., 120.)).Shape(),
              BRepPrimAPI_MakeCone(50., 0., 130.).Shape(), BRepPrimAPI_MakeCylinder(42.5, 100.).Shape()]
    for name, solid in zip(names, solids):
        if not BRepCheck_Analyzer(solid).IsValid():
            raise RuntimeError(f"示例几何无效: {name}")
        writer = STEPControl_Writer()
        if writer.Transfer(solid, STEPControl_AsIs) != IFSelect_RetDone:
            raise RuntimeError(f"STEP 转换失败: {name}")
        if writer.Write(str(directory/f"{name}.step")) != IFSelect_RetDone:
            raise RuntimeError(f"STEP 写入失败: {name}")
    positions = [(-200, -100), (0, -100), (200, -100), (-200, 140), (0, 140), (200, 140)]
    colors = [[210, 40, 35], [40, 180, 45], [30, 65, 200], [210, 210, 30], [175, 40, 180], [230, 120, 30]]
    rotations = [[0, 0, 15], [0, 0, -12], [0, 0, 17], [90, 0, 12], [0, 0, 0], [0, 0, 0]]
    scene = copy.deepcopy(SCENE_DEFAULTS)
    scene["objects"] = [{"name": name, "file": f"{name}.step", "position_mm": list(position),
                         "rotation_deg": rotation, "color_rgb": color}
                        for name, position, rotation, color in zip(names, positions, rotations, colors)]
    path = directory/"scene.json"
    path.write_text(json.dumps(scene, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent/"models"/"examples")
    args = parser.parse_args()
    print(generate(args.output))
