from Node import Node

class AnchorNode(Node):
    # Fixed nodes do not change their position estimation based on their measurements
    def handle_measurement(self, d, recv_particles) -> None:
        pass

    # Fixed nodes do not change their position estimation based on their measurements
    def handle_passive_measurement(self, r, recv_particles_a, recv_particles_b) -> None:
        pass

    # The fixed node does know its position and just returns this as a single particle (this means 100% probability)
    def get_particles(self) -> List[Tuple[float, float]]:
        return [self.pos]

    def get_particles_for_exchange(self) -> List[Tuple[float, float]]:
        return self.get_particles()
