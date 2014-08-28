# id of object, must be unique (this is the id outside of the database).
int32 id

# name of object
string name

# type: "edge" or "vertex"
int32 type
int32 VERTEX = 0
int32 EDGE = 1

# for edges: start and end vertices
# for vertices: not used
int32[] references

