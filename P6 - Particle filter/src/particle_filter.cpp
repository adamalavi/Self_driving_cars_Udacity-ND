/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  for(int i=0; i<num_particles; i++)
  {
      Particle p;
      p.id = i;
      p.x = dist_x(gen);
      p.y = dist_y(gen);
      p.theta = dist_theta(gen);
      p.weight = 1.0;
      particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
   
  for(int i=0; i<num_particles; i++)
  {
      if(fabs(yaw_rate)<0.00001)
      {
          // Different calculations for zero yaw_rate to prevent divide by zero error
          particles[i].x += velocity*delta_t*cos(particles[i].theta);
          particles[i].y += velocity*delta_t*sin(particles[i].theta);
      }
      else
      {
          double term = velocity/yaw_rate;
          particles[i].x += term*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
          particles[i].y += term*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
          particles[i].theta += yaw_rate*delta_t;
      }
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for(unsigned int i=0;i<observations.size();i++)
  {
      LandmarkObs o = observations[i];
      // Initializing the min value as a very high value
      double min_dist = std::numeric_limits<double>::max();
      int min_id = 9999;
      // Looping through every prediction to find the one closest to the observation
      for(unsigned int j=0; j<predicted.size(); j++)
      {
          LandmarkObs p = predicted[j];
          double distance = dist(o.x, o.y, p.x, p.y);
          if(distance<min_dist)
          {
              min_dist = distance;
              min_id = p.id;
          }
      }
      observations[i].id = min_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for(int i=0; i<num_particles; i++)
  {
      double p_x = particles[i].x;
      double p_y = particles[i].y;
      double p_theta = particles[i].theta;
      double o_x, o_y, pr_x, pr_y;
      // Rejecting all the landmarks which are not in the sensor range of the particle
      vector <LandmarkObs> predictions;
      for(unsigned int j=0; j<map_landmarks.landmark_list.size(); j++)
      {
          double lm_x = map_landmarks.landmark_list[j].x_f;
          double lm_y = map_landmarks.landmark_list[j].y_f;
          double lm_id = map_landmarks.landmark_list[j].id_i;
          if (fabs(lm_x - p_x) <= sensor_range && fabs(lm_y - p_y) <= sensor_range)
              predictions.push_back(LandmarkObs{ (int)lm_id, lm_x, lm_y });
      }
      
      // Transforming from vehicle to map coordinates
      vector <LandmarkObs> transformed;
      for (unsigned int j=0; j<observations.size(); j++) 
      {
          double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
          double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
          transformed.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
      }
      
      // Finding the closest landmarks to the observed landmarks
      dataAssociation(predictions, transformed);
      // Finding the coordinates of the observed and predicted landmarks
      for (unsigned int j=0; j<transformed.size(); j++) 
      {
          o_x = transformed[j].x;
          o_y = transformed[j].y;
          for(unsigned int k=0; k<predictions.size(); k++)
          {
              if(predictions[k].id == transformed[j].id)
              {
                  pr_x = predictions[k].x;
                  pr_y = predictions[k].y;
              }
          }
      }
      
      double s_x = std_landmark[0];
      double s_y = std_landmark[1];
      // Multi-variate Gaussian formula 
      double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );

      // product of this obersvation weight with total observations weight
      particles[i].weight *= obs_w;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<Particle> new_particles;
  vector<double> weights;
  double maxWeight = std::numeric_limits<double>::min();
  
  for (int i = 0; i < num_particles; i++) 
  {
    weights.push_back(particles[i].weight);
    if(particles[i].weight > maxWeight) 
    {
        maxWeight = particles[i].weight;
    }
  }
  // Resampling wheel algorithm
  std::uniform_real_distribution <double> distDouble(0.0, maxWeight);
  std::uniform_int_distribution <int> distInt(0, num_particles - 1);
  int index = distInt(gen);
  double beta = 0.0;
  vector<Particle> resampled;
  for(int i = 0; i < num_particles; i++) 
  {
      beta = beta + distDouble(gen) * 2.0;
      while(beta > weights[index]) 
      {
          beta = beta - weights[index];
          index = (index + 1) % num_particles;
      }
      resampled.push_back(particles[index]);
  }
  particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}