import numpy as np
from dataset.nsrm.configs import ParameterSettings as settings

class SceneGraph(object):
    def __init__(self,objects, config):
        self.objects = [o.__dict__ for o in objects]
        self.positions = [o.pos for o in objects]
        self.directions = config.get('directions', settings.DIRECTIONS)
        self.eps = config.get('relation_epsilon', 0.075)
        self.relationships = self.find_relations()

    def find_relations(self):
        num_obj = len(self.objects)
        relationships = {}
        for dir, vec in self.directions.items():
            relationships[dir] = []
            for i in range(num_obj):
                coords_i = self.positions[i]
                related_i = []
                for j in range(num_obj):
                    if j==i:
                        continue
                    coords_j = self.positions[j]
                    diff = np.array(coords_j) - np.array(coords_i)
                    dot = np.dot(np.array(vec), diff)
                    if dot > self.eps:
                        related_i.append(j)
                relationships[dir].append(related_i)
        return relationships

        
    def update(self,positions):
        self.positions = positions
        self.relationships = self.find_relations()
