# id of lama object in the database, will be unique.
# When adding a vertex into the database, this id will be ignored.
int32 id

# id of lama object in the world (usage is implentation specific).
int32 id_in_world

# name of object
string name

# type: "edge" or "vertex"
int32 type
int32 VERTEX = 0
int32 EDGE = 1

# for edges: start and end vertices
# for vertices: not used
int32[] references

