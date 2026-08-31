uint16 origin_tag_id           # tag id
float32 origin_tag_size        # side length of the black square [mm]
float32 origin_tag_pos_z       # height of the tag center above the ground [mm]
float32 origin_tag_pos_deg     # angle relative to the ground [deg]
float32 default_tag_size       # side length of the black square (other than origin tag) [mm]
float32 minimum_viewing_angle  # minimum angle between optical axis and tag surface [degree]
uint32 minimum_tag_area        # minimum tag area in pixels^2. Any tags smaller than that are ignored [px*px]