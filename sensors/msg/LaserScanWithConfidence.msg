# Single scan from a planar laserscan, with a confidence associated with each "range" measurement

Header header                     # timestamp in the header is the acquisition time of 
                                  # the first ray in the scan.
                                  #
                                  # in frame frame_id, angles are measured around 
                                  # the positive Z axis (counterclockwise, if Z is up)
                                  # with zero angle being forward along the x axis
                                  
sensor_msgs/msg/LaserScan scan    # the laserscan information
float32[] confidences             # the confidence values, ordered just 
                                  # as the scan/ranges values

