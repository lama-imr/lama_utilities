/*
 * Short description.
 *
 * Long description:
 * - description of general role
 * - description of used actions with associated result (DONE when, FAIL if...)
 *
 * Interaction with the map (created by this jockey):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter/Setter: VectorLaserScan, jockey_name + "_laser_descriptor"
 *
 * Interaction with the map (created by other jockeys):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter: VectorLaserScan, "laser_descriptor"
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - sensor_msgs::LaserScan, "~/base_scan", 360-deg laser-scan.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 * - nav_msgs::Pose, "~/pose", robot pose
 *
 * Services used (other than map-related):
 * - service type, server default name, description
 * - polygon_matcher::PolygonDissimilarity, "~/dissimilarity_server", used to
 *    compare all known places with the current place
 *
 * Parameters:
 * - name, type, default name, description
 */


