
**18/09/2025**
*Kaue Oliveira*

# TODO (1): Architectural changes to the engine.

The following should be refactored for better modularity of the engine: 

* player.cpp

    * Move the PM_ functions for wishvec player controller into it's own separate p_move.cpp source file.
    >   [!IMPORTANT] 
    >   Ensure that all relative functions are also moved, e.g. ClipVelocity(). and that we don't break the order of 
    >   operations in which the UpdatePlayerMove() function is structured. 
    
    * Rename player.cpp to p_update.cpp since the remaining methods will all be related to updating player state.

* entities/ 
    
    * This folder is currently empty. But it's best to think of a clean modular structure that adhere's to the **Data Oriented Design**
      approach, such that any entity that a user would like to add can be easily programmed.
    > [!TIP]
    >   I believe that a potential method in solving this problem would be to write an entity handler that takes all  entity data from the map parser,
    >   then within the entity handling system we can assign a base component structure to all entities, and write some custom scripted logic using this
    >   structure.
    >
    >``` cpp   
    > // Example code for the structure of defining entities within the engine.
    >   typedef struct Entity {
    >       std::unordered_map<std::string, std::string> properties;
    >       std::vector<Brush> brushes;
    >   } Entity;
    >
    >   typedef struct Map {
    >       std::vector<Entity> entities;
    >   } Map;
    > 
    > // Then we can follow a similar structure to the current info_player_start parsing for non brush entities.
    >   std::vector<FooEntity> ScriptFooEntity(const Map &map) {
    >       std::vector<FooEntity> entity;
    >           for (auto &entity : map.entities) {
    >               auto entClass = entity.properties.find("classname");
    >               if (entClass != entity.properties.end() && entClass->second == "specific_classname") {
    >                   // Here we can define variables that will match certain entity properyies parsed from the map file.
    >                   auto entity_origin = entity.properties.find("origin");
    >                   if (entity_origin != entity.properties.end()) {
    >                       // Implementation of entity property goes here and so on...
    >                   }
    >               }
    >           }  
    >   }  
    >```   

    * Maybe it will be more interesting to add an abstraction where we populate some sort of description structure for the entities, so we can write a function that just extracts 
      the information found for each entity's properties and populate some sort of entity.desc field. This way we can script the entities by refering them to what they are directly. 
      so if need be we can point entities to each other in the map editor directly and have it be automatically scripted. 


    # TODO (2): Change certain movement feel to player movment code.

    * Currently the strafing logic is correct and works fine. But the ramp surf is a bit buggy, there is an issue when changing camera directions on a   ramp that we can set the movement of the player to start surfing backwards when aiming up a ramp instead of upwards like in src games. I need to   find a balance between the quake strafe logic and the source engine handling of slope surfing.



