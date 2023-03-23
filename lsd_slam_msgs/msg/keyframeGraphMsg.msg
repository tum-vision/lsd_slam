# data as serialization of sim(3)'s: (int id, float[7] camToWorld)
uint32 numFrames
uint8[] frameData


# constraints (int from, int to, float err)
uint32 numConstraints
uint8[] constraintsData
