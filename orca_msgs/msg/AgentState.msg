# This represents the ORCA data of any agent in the world detected by laser scan and to be used by the self-agent.

# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The velocity in this message should be specified in the coordinate frame given by header.frame_id.


Header header
int64[] agent_ID
DetectedEntity[] data
