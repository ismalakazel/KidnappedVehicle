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
    
    // Mark program as initialized
    is_initialized = true;

    // Set number of particles
    num_particles = 100;
    
    default_random_engine gen;
    
    // Set normal distribution for x, y and theta
    normal_distribution<double> xdist(x, std[0]);
    normal_distribution<double> ydist(y, std[1]);
    normal_distribution<double> tdist(theta, std[2]);
    
    for (int n = 0; n <= num_particles; n++) {
        
        // Create particle
        Particle particle;
        particle.id = n;
        particle.x = xdist(gen);
        particle.y = ydist(gen);
        particle.theta = tdist(gen);
        particle.weight = 1.0;
        
        // Push to particle set
        particles.push_back(particle);
    }
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    
    default_random_engine gen;

    // Set normal distribution for x, y and theta
    normal_distribution<double> xdist(0, std_pos[0]);
    normal_distribution<double> ydist(0, std_pos[1]);
    normal_distribution<double> tdist(0, std_pos[2]);
    
    for (Particle &p: particles) {
        
        // Predict particle state
        if (fabs(yaw_rate) < 0.00001) {
            
            // When vehicle is driving straight
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        } else {
            
            // When vehicle is making turns
            p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        }
        
        // Set particle noise
        p.x += xdist(gen);
        p.y += ydist(gen);
        p.theta += tdist(gen);
    }
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
    
    for (LandmarkObs &o: observations) {
        
        // Distance threshold
        double gap = numeric_limits<long>::max();
        
        for (LandmarkObs &p: predicted) {
            
            // Euclidean distance between observation and prediction
            double distance = dist(o.x, o.y, p.x, p.y);
            
            if (distance <= gap) {
                
                // Associate observation with prediction
                o.id = p.id;
                
                // Future associations must be equal or less than last associated gap
                gap = distance;
            }
        }
    }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

    // Update each particle weight
    for (Particle &p: particles) {

        // Create prediction and observation lists for later association
        vector<LandmarkObs> predictions;
        vector<LandmarkObs> observed;
        
        for (Map::single_landmark_s m: map_landmarks.landmark_list) {
            
            // Euclidean distance between particle and landmark
            double distance = dist(p.x, p.y, m.x_f, m.y_f);
            
            if (distance <= sensor_range) {
                
                // Create landmark prediction
                LandmarkObs prediction;
                prediction.id = m.id_i;
                prediction.x = m.x_f;
                prediction.y = m.y_f;
                
                // Push to predictions set
                predictions.push_back(prediction);
            }
        }
        
        for (LandmarkObs obs: observations) {
            
            // Create landmark observation
            LandmarkObs converted;
            
            // Convert observation to map coordinate
            converted.x = cos(p.theta) * obs.x - sin(p.theta) * obs.y + p.x;
            converted.y = sin(p.theta) * obs.x + cos(p.theta) * obs.y + p.y;
            
            // Push to landmark observation set
            observed.push_back(converted);
        }
        
        // Reset particle weight
        double weight = 1.0;
        
        // Find associated data
        dataAssociation(predictions, observed);

        // Find association
        for (LandmarkObs &obs: observed) {
            for (LandmarkObs &p: predictions) {
                if (obs.id != p.id) continue;
                
                // Update weight
                double x = std_landmark[0];
                double y = std_landmark[1];
                weight *= exp(-(pow(obs.x - p.x, 2) / (2 * x * x) + pow(obs.y - p.y, 2) / (2 * y * y))) / 2 * M_PI * x * y;
            }
        }
        
        // Update particle weight
        p.weight = weight;
    }
}


void ParticleFilter::resample() {
    
    // Clear weights before resampling
    this->weights.clear();
    
    // Random number generator
    default_random_engine gen;
    
    for (Particle &p: particles) {
        
        // Push to weight set
        weights.push_back(p.weight);
    }
    
    // Create weighted distribution
    discrete_distribution<int> distribution(this->weights.begin(), this->weights.end());
    
    vector<Particle> particles;
    for (Particle particle: this->particles) {
        Particle p = this->particles[distribution(gen)];
        p.weight = 1.0;
        particles.push_back(p);
    }
    
    // Update particles
    this->particles = particles;
}


Particle SetAssociations(Particle& particle, const std::vector<int>& associations, const std::vector<double>& sense_x, const std::vector<double>& sense_y) {
    
    // Clear associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();
    
    // Associate particles
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    
    return particle;
}


string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}


string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}


string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
