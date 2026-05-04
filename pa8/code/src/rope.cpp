#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // DONE (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D step = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; ++i) {
            Mass *m;
            if (i == num_nodes - 1)
                m = new Mass(end, node_mass, false);
            else
                m = new Mass(start + step * i, node_mass, false);
            masses.push_back(m);
        }

        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }

        for (int i = 1; i < num_nodes; ++i) {
            springs.push_back(new Spring(masses[i - 1], masses[i], k));
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
            {
                // DONE (Part 2): Use Hooke's law to calculate the force on a node
                Vector2D dir = s->m2->position - s->m1->position;
                double dis = dir.norm();

                Vector2D force = -s->k * dir / dis * (dis - s->rest_length);
                s->m1->forces -= force;
                s->m2->forces += force;
            }

        for (auto &m : masses)
            {
                if (!m->pinned)
                    {
                        // DONE (Part 2): Add the force due to gravity, then compute the new velocity and position

                        m->last_position = m->position;
                        // explicit
                        // m->position += m->velocity * delta_t;

                        Vector2D a = m->forces / m->mass + gravity;
                        m->velocity += a * delta_t;

                        // semi-implicit
                        m->position += m->velocity * delta_t;

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
                // DONE (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
                Vector2D dir = s->m2->position - s->m1->position;
                double dis = dir.norm();

                Vector2D force = -s->k * dir / dis * (dis - s->rest_length);
                s->m1->forces -= force;
                s->m2->forces += force;
            }

        for (auto &m : masses)
            {
                if (!m->pinned)
                    {
                        Vector2D temp_position = m->position;
                        // TODO (Part 3.1): Set the new position of the rope mass
                        Vector2D a = m->forces / m->mass + gravity;
                        m->position = temp_position * 2 - m->last_position
                                      + a * delta_t * delta_t;
                        m->last_position = temp_position;

                        // TODO (Part 4): Add global Verlet damping
                    }
                m->forces = Vector2D(0, 0);
            }
    }
}
