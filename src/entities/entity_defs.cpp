#include "entity_defs.h"

#include <cstring>

namespace EntityDefs {
    namespace {

        static constexpr EntityDef kEntityDefs[] = {
            { "worldspawn", EntityKind::Brush, ClassMatch::Exact, CollisionRole::Static, true, true, true, false, RuntimeBehavior::None },

            { "trigger_once", EntityKind::Brush, ClassMatch::Exact, CollisionRole::Trigger, false, false, false, false, RuntimeBehavior::TeleportTrigger },
            { "trigger_multiple", EntityKind::Brush, ClassMatch::Exact, CollisionRole::Trigger, false, false, false, false, RuntimeBehavior::TeleportTrigger },
            { "trigger_boost", EntityKind::Brush, ClassMatch::Exact, CollisionRole::Trigger, false, false, false, false, RuntimeBehavior::BoostVolume },

            { "func_boost", EntityKind::Brush, ClassMatch::Exact, CollisionRole::Trigger, false, false, false, false, RuntimeBehavior::BoostVolume },
            { "func_clip", EntityKind::Brush, ClassMatch::Prefix, CollisionRole::Static, false, false, false, false, RuntimeBehavior::None },
            { "func_detail", EntityKind::Brush, ClassMatch::Prefix, CollisionRole::NoCollide, true, true, true, false, RuntimeBehavior::None },
            { "func_group", EntityKind::Brush, ClassMatch::Exact, CollisionRole::NoCollide, true, true, true, false, RuntimeBehavior::None },
            { "func_physics", EntityKind::Brush, ClassMatch::Prefix, CollisionRole::Dynamic, false, true, false, true, RuntimeBehavior::PhysicsBody },

            { "ent_physx", EntityKind::Brush, ClassMatch::Exact, CollisionRole::Dynamic, false, true, false, true, RuntimeBehavior::PhysicsBody },

            { "check_point", EntityKind::Point, ClassMatch::Exact, CollisionRole::Unknown, false, false, false, false, RuntimeBehavior::CheckPoint },
            { "info_player_start", EntityKind::Point, ClassMatch::Exact, CollisionRole::Unknown, false, false, false, false, RuntimeBehavior::None },
            { "info_obj_spawn", EntityKind::Point, ClassMatch::Exact, CollisionRole::Unknown, false, false, false, false, RuntimeBehavior::None },

            { "light_brush", EntityKind::Brush, ClassMatch::Exact, CollisionRole::NoCollide, true, true, true, false, RuntimeBehavior::None },
            { "light", EntityKind::Point, ClassMatch::Exact, CollisionRole::Unknown, false, false, false, false, RuntimeBehavior::None },
            { "light_point", EntityKind::Point, ClassMatch::Exact, CollisionRole::Unknown, false, false, false, false, RuntimeBehavior::None },
        };

        static bool Matches(const EntityDef& def, const std::string& classname) {
            if (def.match == ClassMatch::Exact) {
                return classname == def.classname;
            }

            const size_t prefixLen = std::strlen(def.classname);
            return classname.size() >= prefixLen && classname.compare(0, prefixLen, def.classname) == 0;
        }

    } // namespace

    const EntityDef* Find(const std::string& classname) {
        for (const EntityDef& def : kEntityDefs) {
            if (Matches(def, classname)) {
                return &def;
            }
        }
        return nullptr;
    }

    const EntityDef* Find(const Entity& entity) {
        auto it = entity.properties.find("classname");
        if (it == entity.properties.end()) {
            return nullptr;
        }
        return Find(it->second);
    }

    const char* Classname(const Entity& entity) {
        auto it = entity.properties.find("classname");
        return it != entity.properties.end() ? it->second.c_str() : "";
    }

} // namespace EntityDefs
