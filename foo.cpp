
// Associate each of the transformed observations with the nearest landmark && Update the weights.
for (int i = 0; i < trans_obs.size(); i++)
{
  double x = trans_obs[i].x;
  double y = trans_obs[i].y;
  double x_diff = x - surroundinglandmarks[0].x; // Difference of x between transformed observations and the first map landmarks.
  double y_diff = y - surroundinglandmarks[0].y; // Difference of y between transformed observations and the first map landmarks.

  double min_dis = dist(x, y, surroundinglandmarks[0].x, surroundinglandmarks[0].y);

  int id = surroundinglandmarks[0].id;
  double x_lm = surroundinglandmarks[0].x;
  double y_lm = surroundinglandmarks[0].y;

  // Calculate the difference between transformed observations and other landmarks and find the associated(nearest) landmark.
  for (int i = 1; i < surroundinglandmarks.size(); i++)
  {
    double dis = dist(x, y, surroundinglandmarks[i].x, surroundinglandmarks[i].y);

    if (dis < min_dis)
    {
      min_dis = dis;
      x_diff = x - surroundinglandmarks[i].x;
      y_diff = y - surroundinglandmarks[i].y;

      id = surroundinglandmarks[i].id;
      x_lm = surroundinglandmarks[i].x;
      y_lm = surroundinglandmarks[i].y;
    }
  }
  // This line for debugging.
  //cout << "obs_original = (" << observations[i].x << ", " << observations[i].y << "), obs_trans = (" << x << ", " << y << "), ass_lm_id = " << id << ", " << " ass_lm = (" << x_lm << ", " << y_lm << ")" << endl;

  double exponent = 0.5 * (pow(x_diff / std_x, 2) + pow(y_diff / std_y, 2));
  weight = weight * exp(-exponent) / (2 * M_PI * std_x * std_y);

}
particles[i].weight = weight; // Update the weight of each particle.
//cout <<" particle_weight = " << particles[i].weight << endl;// This line for debugging.
weights[i] = weight; // Update the vector of weights.
