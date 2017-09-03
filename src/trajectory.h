//
// Created by Thomas Antony on 9/03/17.
//

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

/* Helper functions provided by Udacity */
double distance (double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const Map& map)
{
	int next_wp = NextWaypoint(x,y, theta, map.x,map.y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = map.x.size()-1;
	}

	double n_x = map.x[next_wp]-map.x[prev_wp];
	double n_y = map.y[next_wp]-map.y[prev_wp];
	double x_x = x - map.x[prev_wp];
	double x_y = y - map.y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-map.x[prev_wp];
	double center_y = 2000-map.y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(map.x[i],map.y[i],map.x[i+1],map.y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const Map& map)
{
	int prev_wp = -1;

	while(s > map.s[prev_wp+1] && (prev_wp < (int)(map.s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map.x.size();

	double heading = atan2((map.y[wp2]-map.y[prev_wp]),(map.x[wp2]-map.x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map.s[prev_wp]);

	double seg_x = map.x[prev_wp]+seg_s*cos(heading);
	double seg_y = map.y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/** Coordinate transformations **/
inline const tuple<double, double> GlobalToLocal(const tuple<double, double> input, const tuple<double, double, double> ref)
{
  // First find relative coordinates and then rotate them
  double x, y, x0, y0, theta0;
  std::tie(x, y) = input;
  std::tie(x0, y0, theta0) = ref;

  // Translate
  x = x - x0;
  y = y - y0;

  // Rotate
  return std::make_tuple(x * cos(theta0) + y * sin(theta0),
                        -x * sin(theta0) + y * cos(theta0));
}

inline const tuple<double, double> LocalToGlobal(const tuple<double, double> input, const tuple<double, double, double> ref)
{
  // First find relative coordinates and then rotate them
  double x, y, x0, y0, theta0;
  std::tie(x, y) = input;
  std::tie(x0, y0, theta0) = ref;

  // Rotate and translate
  return std::make_tuple(x0 + x * cos(theta0) - y * sin(theta0),
                         y0 + x * sin(theta0) + y * cos(theta0));
}

/** Trajectory Generator **/
const Trajectory GenerateTrajectory(const Command& cmd,
                                    const Car& ego,
                                    const Map& map,
                                    const Trajectory& prev_path)
{
  vector<double> spline_anchor_x, spline_anchor_y;

  const auto num_prev_points = prev_path.size();
  // Reference state for coordinate transformation
  // Start with car state as reference
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = ego.yaw;

  double ego_s = ego.s;
  if (num_prev_points > 2) {
    // Else start at the end of prior waypoints
    // and use them as reference for coordinate transform
    ref_x = prev_path.x[num_prev_points-1];
    ref_y = prev_path.y[num_prev_points-1];

    // Estimate past heading
    double prev_wp_x = prev_path.x[num_prev_points-2];
    double prev_wp_y = prev_path.y[num_prev_points-2];
    ref_yaw = atan2(ref_y - prev_wp_y,
                    ref_x - prev_wp_x);

    spline_anchor_x.push_back(prev_path.x[num_prev_points-2]);
    spline_anchor_y.push_back(prev_path.y[num_prev_points-2]);
    ego_s = prev_path.end_path_s; // Helpful for making smooth trajectories
  }
  // position at t
  spline_anchor_x.push_back(ref_x);
  spline_anchor_y.push_back(ref_y);

  // Add equally spaced points in Frenet space
  for(int i = 0; i < 3; i++)
  {
    auto wp = getXY(ego_s + anchor_spacing*(i+1),
                    (2+4*cmd.lane),
                    map);
    spline_anchor_x.push_back(wp[0]);
    spline_anchor_y.push_back(wp[1]);
  }

  // Convert spline points to local coordinates
  auto ref_state = std::make_tuple(ref_x, ref_y, ref_yaw);
  for(int i=0; i<spline_anchor_x.size(); i++)
  {
    auto x = spline_anchor_x[i];
    auto y = spline_anchor_y[i];
    std::tie(x, y) = GlobalToLocal(std::make_tuple(x, y), ref_state);
    spline_anchor_x[i] = x;
    spline_anchor_y[i] = y;
  }

  tk::spline s;
  // Fit spline to anchor points
  s.set_points(spline_anchor_x, spline_anchor_y);

  // Copy old waypoints first
  Trajectory output;

  for(int i = 0; i < num_prev_points; i++)
  {
    output.x.push_back(prev_path.x[i]);
    output.y.push_back(prev_path.y[i]);
    auto sd = getFrenet(prev_path.x[i], prev_path.y[i], ego.yaw, map);
    output.s.push_back(sd[0]);
    output.d.push_back(sd[1]);
  }

  // Find waypoint spacing based on speed
  const auto target_x = anchor_spacing;
  const auto target_y = s(target_x);
  const auto target_dist = sqrt(target_x*target_x + target_y*target_y);
  double N = target_dist / (timestep*cmd.speed);

  // Find new points and transform back to global space
  double last_x = 0;
  for(int i = 1; i <= 50 - num_prev_points; i++)
  {
    // double wp_x = last_x + anchor_spacing/num_points;
    double wp_x = last_x + target_x/N;
    double wp_y = s(wp_x);

    last_x = wp_x;
    tie(wp_x, wp_y) = LocalToGlobal(make_tuple(wp_x, wp_y), ref_state);

    output.x.push_back(wp_x);
    output.y.push_back(wp_y);
  }
  return output;
}
#endif //TRAJECTORY_H
