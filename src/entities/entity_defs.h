#pragma once

#include "../utils/map_types.h"

#include <string>

namespace EntityDefs {

    enum class EntityKind {
        Point,
        Brush,
    };

    enum class CollisionRole {
        Unknown,
        Static,
        Dynamic,
        Trigger,
        NoCollide,
    };

    enum class RuntimeBehavior {
        None,
        CheckPoint,
        BoostVolume,
        TeleportTrigger,
        PhysicsBody,
    };

    enum class ClassMatch {
        Exact,
        Prefix,
    };

    struct EntityDef {
        const char* classname = "";
        EntityKind kind = EntityKind::Brush;
        ClassMatch match = ClassMatch::Exact;
        CollisionRole collision = CollisionRole::Unknown;
        bool renderBrushGeometry = true;
        bool devRenderBrushGeometry = true;
        bool bakeLightGeometry = true;
        bool dynamicRenderMesh = false;
        RuntimeBehavior behavior = RuntimeBehavior::None;
    };

    const EntityDef* Find(const std::string& classname);
    const EntityDef* Find(const Entity& entity);
    const char* Classname(const Entity& entity);

} // namespace EntityDefs
