#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        Mass *mass = new Mass(start, node_mass, false);
        masses.push_back(mass);
        for (int i = 1; i < num_nodes-1; i++)
        {
            Mass *mass = new Mass((start*(num_nodes-1-i)+end*i)/(num_nodes-1), node_mass, false);
            masses.push_back(mass);
        }
        mass = new Mass(end, node_mass, false);
        masses.push_back(mass);

        for (int i = 1; i < num_nodes; i++)
        {
            Spring *spring = new Spring(masses[i-1], masses[i], k);
            springs.push_back(spring);
        }

//       Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            double distance = (s->m1->position - s->m2->position).norm();
            Vector2D forces = - s->k * (s->m1->position - s->m2->position) / distance * (distance - s->rest_length);
            s->m1->forces += forces;
            s->m2->forces += -forces;

            Vector2D dampingForces = damping * (s->m1->position - s->m2->position) / distance * (s->m1->velocity - s->m2->velocity) * (s->m1->position - s->m2->position) / distance;
            s->m1->forces += dampingForces;
            s->m2->forces -= dampingForces;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity;
                m->velocity += m->forces / m->mass * delta_t;
                m->position += m->velocity *delta_t;

                // TODO (Part 2): Add global damping

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            double distance = (s->m1->position - s->m2->position).norm();
            Vector2D forces = - s->k * (s->m1->position - s->m2->position) / distance * (distance - s->rest_length);
            s->m1->forces += forces;
            s->m2->forces += -forces;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                m->forces += gravity;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->position += (1 - 0.00005) * (m->position - m->last_position) + m->forces / m->mass * delta_t * delta_t;
                // TODO (Part 4): Add global Verlet damping
                m->last_position = temp_position;
            }

            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerletConstraints(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D direction = s->m1->position - s->m2->position;
            double distance = direction.norm();
            direction /= distance;
            //Vector2D forces = - s->k * (s->m1->position - s->m2->position) / distance * (distance - s->rest_length);
            Vector2D constra = - 0.5 * (distance - s->rest_length) * direction;
            s->m1->forces += constra;
            s->m2->forces += -constra;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->position += (1 - 0.00005) * (m->position - m->last_position) + gravity / m->mass * delta_t * delta_t + m->forces;
                // TODO (Part 4): Add global Verlet damping
                m->last_position = temp_position;
            }

            m->forces = Vector2D(0, 0);
        }
    }
}
