/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Updated on: Oct 2018
 *      Author: Noah Fisher
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on
  // estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.  Add random
  // Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this
  // file).
  //
  // --- Code below:
  default_random_engine gen;
  // @hyperparameter:
  //   guess for total number of particles. Can play with this later to see how changing orders of
  //   magnitude impacts the error.
  num_particles = 10;

  // Create normal (Gaussian) distributions for x, y, theta given std array
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Initialize particles with noise
  for (int i = 0; i < num_particles; ++i) {
    Particle p = Particle();

    // Sample from normal distrubtions
    p.id = i;
    p.theta = dist_theta(gen);
    p.weight = 1;
    p.x = dist_x(gen);
    p.y = dist_y(gen);

    particles.push_back(p);
    weights.push_back(p.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/
  //
  //
  // --- Code below
  default_random_engine gen;

  for(int i = 0; i < num_particles; i++) {
    Particle part = particles[i];
    double new_x;
    double new_y;
    double new_theta;

    if(yaw_rate == 0) {
      new_theta = part.theta;
      new_x = part.x + velocity * delta_t * cos(part.theta);
      new_y = part.y + velocity * delta_t * sin(part.theta);
    } else {
      double theta1 = yaw_rate * delta_t;
      new_theta = part.theta + theta1;
      new_x = part.x + ( velocity / yaw_rate ) * (sin(part.theta + theta1) - sin(part.theta));
      new_y = part.y + ( velocity / yaw_rate ) * (cos(part.theta) - cos(part.theta + theta1));
    }

    // Create normal (Gaussian) distributions for x, y, theta given std array
    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);

    part.theta = dist_theta(gen);
    part.x = dist_x(gen);
    part.y = dist_y(gen);
    part.weight = 1.0; // initialize weights to 1
    weights[i] = 1.0;  // keep weights in sync with particles
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> landmarks, std::vector<LandmarkObs>& observations) {
  // TODO: Find the landmark measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.
  //
  // --- Code below
  // From the Code Video:
  // > takes input 2 vectors.
  // > the predicted measurements between one particular particle and all the map landmarks within
  //   sensor range.
  // > actual sensor measurements gathered from the lidar.
  //
  // assign each sensor observation the map landmark id associated with it (associated by nearest
  // neighbor)

  // FIXME: this may fail if there are no map observations nearby...
  for(int i = 0; i < observations.size(); i++) {
    LandmarkObs nearestLandmark;

    double minPredDist = numeric_limits<const double>::infinity();
    // find the closest observation for each prediction and update the landmark id
    for(int j = 0; j < landmarks.size(); j++) {
      double tmpDst = dist(observations[i].x, observations[i].y, landmarks[j].x, landmarks[j].y);
      //cout << "tmpDst= " << tmpDst << " minPredDist= " << minPredDist << "\t";
      if(tmpDst <= minPredDist) {
        minPredDist = tmpDst;
        nearestLandmark = landmarks[j];
      }
    }
    //cout << "nearestLandmark.id " << nearestLandmark.id << endl;
    // set the observation to the nearest landmark.
    observations[i].id = nearestLandmark.id;
    observations[i].l_x = nearestLandmark.x;
    observations[i].l_y = nearestLandmark.y;
    observations[i].diff_x = nearestLandmark.x - observations[i].x;
    observations[i].diff_y = nearestLandmark.y - observations[i].y;
    //cout << endl;
  }
  //cout << endl;
}

void ParticleFilter::updateWeights(double sensor_range,
                                   double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html
  //
  // --- Code below
  // FIXME
  //
  //  - first, transfer observations into global coordinate system
  //  - find the distance between each observation and all the map landmarks within the sensor
  //  range.
  cout << "Timestep: " << timestep;

  for(int p=0; p < num_particles; p++){
    Particle part = particles[p];
    cout << endl << "Particle: " << "(" << part.x << ", " << part.y << ")" << endl;

    // find landmarks within sensor range of particle
    vector<LandmarkObs> landmark_list;
    for(int i=0; i < map_landmarks.landmark_list.size(); i++){
      double distance = dist(part.x, part.y, map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f);
      if(distance <= sensor_range) {
        LandmarkObs obs;
        obs.id = map_landmarks.landmark_list[i].id_i;
        obs.x = map_landmarks.landmark_list[i].x_f;
        obs.y = map_landmarks.landmark_list[i].y_f;
        landmark_list.push_back(obs);
      }
    }

    // observations are given in car coordinate space, transform to map coordinate space
    vector<LandmarkObs> transformed_observations;
    for(int i=0; i < observations.size(); i++){
      LandmarkObs transformed_obs;
      LandmarkObs obs = observations[i];

      // transform to map space
      transformed_obs.x = part.x + obs.x * cos(part.theta) - obs.y * sin(part.theta);
      transformed_obs.y = part.y + obs.x * sin(part.theta) + obs.y * cos(part.theta);
      transformed_observations.push_back(transformed_obs);
      cout << "Observation(" << obs.x << ", " << obs.y << "). ";
      cout << "Transformed(" << transformed_obs.x << ", " << transformed_obs.y << ") " << endl;
    }

    dataAssociation(landmark_list, transformed_observations);

    // cout << "landmark_lis = " << endl;;
    // for(int i=0; i < transformed_observations.size(); i++){
    //   cout << "(" << transformed_observations[i].l_x << ", " << transformed_observations[i].l_y << ")\t";
    // }
    // cout << endl;

    // update weights
    part.weight = 1.0;
    // cout << "particle weight = ";
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    for(int i=0; i < transformed_observations.size(); i++){
      LandmarkObs obs = transformed_observations[i];
      // difference in landmark observation and sensor observation
      double exponent = pow(obs.x - obs.l_x, 2) / (2 * pow(std_landmark[0], 2)) +
                        pow(obs.y - obs.l_y, 2) / (2 * pow(std_landmark[1], 2));
      double weight = exp( - 0.5 * exponent ) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
      // cout << weight << " \t ";
      part.weight *= weight;
      associations.push_back(obs.id);
      sense_x.push_back(obs.x);
      sense_y.push_back(obs.y);

      cout << "Trans(" << transformed_observations[i].x << ", " << transformed_observations[i].y << ", " << transformed_observations[i].id << ") "
           << "Landm(" << transformed_observations[i].l_x << ", " << transformed_observations[i].l_y << ", " << transformed_observations[i].id << ") "
           << "Weigh(" << part.x - obs.l_x << ", " << part.y - obs.l_y << "), exp(" << exponent << "), weight=" << part.weight << endl;
    }

    cout << endl;
    SetAssociations(part, associations, sense_x, sense_y);
    // cout << " ==> " << part.weight << "; ";
  }

  cout << endl;
  timestep++;
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  //

  default_random_engine gen;
  discrete_distribution<int> distribution(weights.begin(), weights.end());

  vector<Particle> resample_particles;
  for (int i = 0; i < num_particles; i++) {
    resample_particles.push_back(particles[distribution(gen)]);
  }
  particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    // particle:        the particle to assign each listed association, and association's (x,y)
    //                  world coordinates mapping to
    // associations:    the landmark id that goes along with each listed association
    // sense_x:         the associations x mapping already converted to world coordinates
    // sense_y:         the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
