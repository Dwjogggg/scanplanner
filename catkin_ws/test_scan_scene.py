"""Regression checks for actual geometry, reproducibility and on-path obstruction."""
import math
import unittest

from scan_scene import fixed_episode, randomized_episode, obstacle_distance, route_distance


class RandomizedSceneTests(unittest.TestCase):
    def test_fixed_scene_has_three_blockers_on_reference(self):
        scene, reference, feasible = fixed_episode()
        self.assertEqual(scene['name'], 'go2_path_obstacle_test')
        self.assertEqual(len(scene['obstacles']), 22)
        blockers = [o for o in scene['obstacles'] if o.get('on_reference')]
        self.assertEqual(len(blockers), 3)
        for o in blockers:
            self.assertLess(route_distance(o['xy'], reference), 1e-8)
            self.assertGreater(route_distance(o['xy'], feasible), 0.85)

    def test_seed_reproduces_everything(self):
        self.assertEqual(randomized_episode(20261105), randomized_episode(20261105))

    def test_layouts_are_distinct_and_solvable(self):
        hashes, counts, shapes = set(), set(), set()
        for seed in range(100):
            scene, reference, feasible = randomized_episode(seed)
            hashes.add(scene['layout_hash'])
            counts.add(len(scene['obstacles']))
            self.assertGreaterEqual(len(scene['obstacles']), 21)
            for p in [scene['start'], scene['goal']]:
                self.assertGreaterEqual(min(obstacle_distance(p,o) for o in scene['obstacles']), 0.65)
            blockers = [o for o in scene['obstacles'] if o.get('on_reference')]
            self.assertEqual(len(blockers), 3)
            for o in blockers:
                shapes.add(o['shape'])
                self.assertLess(route_distance(o['xy'], reference), 1e-8)
                # The feasible detour must clear every blocker, not reuse its center.
                self.assertGreater(route_distance(o['xy'], feasible), 0.85)
            background = [o for o in scene['obstacles'] if not o.get('on_reference')]
            for route, obstacles in [(reference, background), (feasible, scene['obstacles'])]:
                for a,b in zip(route[:-1],route[1:]):
                    n = max(1,math.ceil(math.dist(a,b)/0.04))
                    for i in range(n+1):
                        p = [a[j]+(b[j]-a[j])*i/n for j in range(2)]
                        self.assertGreaterEqual(min(obstacle_distance(p,o) for o in obstacles),0.64)
        self.assertEqual(len(hashes), 100)
        self.assertGreater(len(counts), 3)
        self.assertEqual(shapes, {'box', 'cylinder'})


if __name__ == '__main__':
    unittest.main()
