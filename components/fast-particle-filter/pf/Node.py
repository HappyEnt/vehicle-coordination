class Node:
    def __init__(self, pos) -> None:
        self.pos = pos

    def handle_measurement(self, d, recv_particles) -> None:
        pass

    def handle_passive_measurement(self, r, recv_particles_a, recv_particles_b) -> None:
        pass

    def get_particles(self) -> List[Tuple[float, float]]:
        return []

    def get_particles_for_exchange(self) -> List[Tuple[float, float]]:
        return []

    def get_pos(self) -> Tuple[float, float]:
        return self.pos

    # This estimates the position based on the list of particles
    def get_estimated_pos(self) -> Optional[Tuple[float, float]]:
        particles = self.get_particles()
        if len(particles) == 0:
            return None

        avg = (0.0, 0.0)
        for p in particles:
            avg = (avg[0] + p[0], avg[1]+ p[1])

        avg = (avg[0] / len(particles), avg[1] / len(particles))
        return avg
