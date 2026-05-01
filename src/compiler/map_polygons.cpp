#include "map_polygons.h"
#include "map_csg.h"
#include "map_entity_props.h"
#include "map_geometry.h"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <vector>

void AppendBrushEntityPolygons(const Entity& entity,
                                      int sourceEntityId,
                                      bool devMode,
                                      bool exteriorOnly,
                                      int* nextLightBrushGroup,
                                      int* nextSourceBrushId,
                                      size_t* sourceFaceCount,
                                      std::vector<MapPolygon>& out) {
    const bool isLightBrush = EntityHasClass(entity, "light_brush");
    const std::string lightBrushTexture = isLightBrush
        ? EncodeLightBrushTextureName(ReadLightBrushColor255(entity))
        : std::string();

    if (!ShouldRenderBrushEntity(entity, devMode)) {
        return;
    }

    int phongEnabled = 0;
    ParseIntProp(entity, "_phong", phongEnabled);
    float phongAngle = 89.0f;
    if (ParseFloatProp(entity, "_phong_angle", phongAngle)) {
        phongEnabled = 1;
    }
    float phongAngleConcave = 0.0f;
    ParseFloatProp(entity, "_phong_angle_concave", phongAngleConcave);
    int phongGroup = 0;
    ParseIntProp(entity, "_phong_group", phongGroup);
    int surfaceLightGroup = 0;
    ParseIntProp(entity, "_surflight_group", surfaceLightGroup);
    int noBounce = 0;
    ParseIntProp(entity, "_nobounce", noBounce);
    float surfLightAttenuation = -1.0f;
    if (ParseFloatProp(entity, "_surflight_atten", surfLightAttenuation)) {
        surfLightAttenuation = std::max(0.0f, surfLightAttenuation);
    }
    int surfLightRescale = -1;
    if (ParseIntProp(entity, "_surflight_rescale", surfLightRescale)) {
        surfLightRescale = (surfLightRescale != 0) ? 1 : 0;
    }
    if (!EntityHasClass(entity, "worldspawn")) {
        int bounceOverride = 1;
        if (ParseIntProp(entity, "_bounce", bounceOverride) && bounceOverride <= 0) {
            noBounce = 1;
        }
    }

    std::vector<MapPolygon> entityPolys;

    for (auto& brush : entity.brushes) {
        const int lightBrushGroup = (isLightBrush && nextLightBrushGroup) ? (*nextLightBrushGroup)++ : -1;
        const int sourceBrushId = nextSourceBrushId ? (*nextSourceBrushId)++ : -1;

        const int nF = (int)brush.faces.size();
        if (nF < 3) continue;

        std::vector<Plane> planes(nF);
        for (int i = 0; i < nF; ++i) {
            planes[i] = brush.faces[i].plane;
        }

        std::vector<std::vector<Vector3>> polys(nF);
        for (int i = 0; i < nF - 2; ++i)
          for (int j = i + 1; j < nF - 1; ++j)
            for (int k = j + 1; k < nF; ++k) {
                Vector3 ip;
                if (!GetIntersection(planes[i], planes[j], planes[k], ip)) continue;
                bool inside = true;
                for (int m = 0; m < nF; ++m) {
                    float d = Vector3DotProduct(brush.faces[m].normal, ip) + planes[m].d;
                    if (d > (float)epsilon) { inside = false; break; }
                }
                if (inside) {
                    polys[i].push_back(ip);
                    polys[j].push_back(ip);
                    polys[k].push_back(ip);
                }
            }

        for (int i = 0; i < nF; ++i) {
            RemoveDuplicatePoints(polys[i], (float)epsilon);
            if (polys[i].size() < 3) continue;

            Vector3 nTB = brush.faces[i].normal;
            SortPolygonVertices(polys[i], nTB);
            RemoveDuplicatePoints(polys[i], CSG_POINT_EPS);
            CleanupClippedPolygon(polys[i], nTB);
            if (polys[i].size() < 3) continue;

            Vector3 n = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
            if (Vector3DotProduct(n, nTB) < 0.f) {
                std::reverse(polys[i].begin(), polys[i].end());
                n = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
            }

            MapPolygon mp;
            mp.verts      = std::move(polys[i]);
            mp.normal     = n;
            mp.facePlaneD = (float)planes[i].d;
            mp.texture    = isLightBrush ? lightBrushTexture : brush.faces[i].texture;
            mp.occluderGroup = lightBrushGroup;
            mp.sourceBrushId = sourceBrushId;
            mp.sourceEntityId = sourceEntityId;
            mp.sourceFaceIndex = i;
            mp.surfaceLightGroup = surfaceLightGroup;
            mp.noBounce = (uint8_t)(noBounce != 0);
            mp.surfLightAttenuation = surfLightAttenuation;
            mp.surfLightRescale = (int8_t)surfLightRescale;
            mp.phong = phongEnabled != 0;
            mp.phongAngle = std::max(1.0f, phongAngle);
            mp.phongAngleConcave = std::max(0.0f, phongAngleConcave);
            mp.phongGroup = phongGroup;
            mp.texAxisU = brush.faces[i].textureAxes1;
            mp.texAxisV = brush.faces[i].textureAxes2;
            mp.offU     = brush.faces[i].offsetX;
            mp.offV     = brush.faces[i].offsetY;
            mp.rot      = brush.faces[i].rotation;
            mp.scaleU   = brush.faces[i].scaleX;
            mp.scaleV   = brush.faces[i].scaleY;
            entityPolys.push_back(std::move(mp));
            if (sourceFaceCount) {
                ++(*sourceFaceCount);
            }
        }
    }

    if (exteriorOnly) {
        std::vector<MapPolygon> exteriorPolys = BuildExteriorPolygons(entityPolys);
        HealTJunctions(exteriorPolys, CSG_POINT_EPS);
        ConvertMapPolygonsTBToWorld(exteriorPolys);
        out.insert(out.end(),
                   std::make_move_iterator(exteriorPolys.begin()),
                   std::make_move_iterator(exteriorPolys.end()));
    } else {
        HealTJunctions(entityPolys, CSG_POINT_EPS);
        ConvertMapPolygonsTBToWorld(entityPolys);
        out.insert(out.end(),
                   std::make_move_iterator(entityPolys.begin()),
                   std::make_move_iterator(entityPolys.end()));
    }
}

std::vector<MapPolygon> BuildMapPolygons(const Map &map, bool devMode)
{
    std::vector<MapPolygon> out;
    int nextLightBrushGroup = 0;
    int nextSourceBrushId = 0;
    size_t sourceFaceCount = 0;
    for (size_t entityIndex = 0; entityIndex < map.entities.size(); ++entityIndex) {
        AppendBrushEntityPolygons(map.entities[entityIndex], (int)entityIndex, devMode, false, &nextLightBrushGroup, &nextSourceBrushId, &sourceFaceCount, out);
    }
    printf("[MapParser] Built %zu polygons from %zu brush faces.\n", out.size(), sourceFaceCount);
    return out;
}

std::vector<MapPolygon> BuildExteriorMapPolygons(const Map &map, bool devMode)
{
    std::vector<MapPolygon> out;
    int nextLightBrushGroup = 0;
    int nextSourceBrushId = 0;
    size_t sourceFaceCount = 0;
    for (size_t entityIndex = 0; entityIndex < map.entities.size(); ++entityIndex) {
        AppendBrushEntityPolygons(map.entities[entityIndex],
                                  (int)entityIndex,
                                  devMode,
                                  true,
                                  &nextLightBrushGroup,
                                  &nextSourceBrushId,
                                  &sourceFaceCount,
                                  out);
    }
    printf("[MapParser] Built %zu entity-local union/exterior polygons from %zu brush faces.\n", out.size(), sourceFaceCount);
    return out;
}
