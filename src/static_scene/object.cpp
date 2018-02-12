#include "object.h"
#include "sphere.h"
#include "triangle.h"

#include <vector>
#include <unordered_map>

using std::vector;
using std::unordered_map;

namespace CMU462 {
	namespace StaticScene {

		// Mesh object //
		Mesh::Mesh(const HalfedgeMesh& mesh, BSDF* bsdf) {

			unordered_map<const Vertex *, int> vertexLabels;
			vector<const Vertex *> verts;

			size_t vertexI = 0;
			for (VertexCIter it = mesh.verticesBegin(); it != mesh.verticesEnd(); it++) {
				const Vertex *v = &*it;
				verts.push_back(v);
				vertexLabels[v] = vertexI;
				vertexI++;
			}

			positions = new Vector3D[vertexI];
			normals = new Vector3D[vertexI];
			for (int i = 0; i < vertexI; i++) {
				positions[i] = verts[i]->position;
				normals[i] = verts[i]->normal;
			}

			for (FaceCIter f = mesh.facesBegin(); f != mesh.facesEnd(); f++) {
				HalfedgeCIter h = f->halfedge();
				indices.push_back(vertexLabels[&*h->vertex()]);
				indices.push_back(vertexLabels[&*h->next()->vertex()]);
				indices.push_back(vertexLabels[&*h->next()->next()->vertex()]);
			}

			// fill primitives vector fot pathtracer
		    const auto num_triangles = indices.size() / 3;
			primitives = new vector<Primitive*>;
			for (size_t i = 0; i < num_triangles; ++i) {
				Triangle *tri = new Triangle(this, indices[i * 3],
					indices[i * 3 + 1],
					indices[i * 3 + 2]);
				(*primitives).push_back(tri);

			}
			this->bsdf = bsdf;

		}

		Mesh::~Mesh()
		{
			for (unsigned int i = 0; i < primitives->size(); i++)
				delete (*primitives)[i];
			delete primitives;
			delete[] positions;
			delete[] normals;
		}

		vector<Primitive*> *Mesh::get_primitives() const
		{
			return primitives;
		}

		BSDF* Mesh::get_bsdf() const {
			return bsdf;
		}

		// Sphere object //

		SphereObject::SphereObject(const Vector3D& o, double r, BSDF* bsdf) {

			this->o = o;
			this->r = r;
			this->bsdf = bsdf;

			primitives = new vector<Primitive*>(1);
			(*primitives).push_back(new Sphere(this, o, r));

		}

		SphereObject::~SphereObject()
		{
			for (unsigned int i = 0; i < primitives->size(); i++)
				delete (*primitives)[i];
			delete primitives;
		}

		vector<Primitive*> *SphereObject::get_primitives() const
		{
			return primitives;
		}

		BSDF* SphereObject::get_bsdf() const {
			return bsdf;
		}


	} // namespace StaticScene
} // namespace CMU462
