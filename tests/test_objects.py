import unittest
from tests.core import TestCore
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
import numpy as np


class TestObjects(TestCore):

    def setUp(self):
        super().setUp()
        self.dynamic_cube = Shape('dynamic_cube')
        self.cubes_under_dummy = Dummy('cubes_under_dummy')
        self.cube0 = Shape('cube0')
        self.dummy = Dummy('dummy')
        self.simple_model = Shape('simple_model')

    def test_equality(self):
        cube2 = Shape('dynamic_cube')
        self.assertEqual(self.dynamic_cube, cube2)

    def test_get_handle(self):
        self.assertGreater(self.dynamic_cube.get_handle(), 0)

    def test_still_exists(self):
        self.assertTrue(self.dynamic_cube.still_exists())
        self.assertFalse(Shape(-1).still_exists())

    def test_object_exists(self):
        yes = Object.exists('dynamic_cube')
        no = Object.exists('dynamic_cubeee')
        self.assertTrue(yes)
        self.assertFalse(no)

    def test_get_set_name(self):
        self.dynamic_cube.set_name('test1')
        self.assertEqual(self.dynamic_cube.get_name(), 'test1')

    def test_get_set_position(self):
        self.cube0.set_position([0.1, 0.1, 0.1], self.cubes_under_dummy)
        self.assertTrue(np.allclose(
            self.cube0.get_position(self.cubes_under_dummy), [0.1, 0.1, 0.1]))
        self.cube0.set_position([0.2, 0.2, 0.2])
        self.assertTrue(np.allclose(self.cube0.get_position(), [0.2, 0.2, 0.2]))

    def test_get_set_orientation(self):
        self.cube0.set_orientation([0.0, 0.0, 0.2], self.cubes_under_dummy)
        self.assertTrue(np.allclose(
            self.cube0.get_orientation(self.cubes_under_dummy),
            [0.0, 0.0, 0.2]))
        self.cube0.set_orientation([0.0, 0.0, 0.3])
        self.assertTrue(np.allclose(
            self.cube0.get_orientation(), [0.0, 0.0, 0.3]))

    def test_get_set_quaternion(self):
        # x, y, z, w
        self.cube0.set_quaternion([1., 0., 0., 0.], self.cubes_under_dummy)
        self.assertTrue(np.allclose(
            self.cube0.get_quaternion(self.cubes_under_dummy),
            [1., 0., 0., 0.]))
        self.cube0.set_quaternion([np.sqrt(0.5), 0, 0., np.sqrt(0.5)])
        self.assertTrue(np.allclose(
            self.cube0.get_quaternion(),
            [np.sqrt(0.5), 0, 0., np.sqrt(0.5)]))

    def test_get_set_parent(self):
        self.dynamic_cube.set_parent(self.dummy)
        parent = self.dynamic_cube.get_parent()
        self.assertEqual(parent, self.dummy)

    def test_get_set_parent_not_in_place(self):
        init_pos = self.dynamic_cube.get_position()
        self.dynamic_cube.set_parent(self.dummy, keep_in_place=False)
        parent = self.dynamic_cube.get_parent()
        self.assertEqual(parent, self.dummy)
        self.assertFalse(np.allclose(
            init_pos, self.dynamic_cube.get_position()))

    def test_get_parent_when_orphan(self):
        parent = self.dummy.get_parent()
        self.assertIsNone(parent)

    def test_get_matrix(self):
        self.assertEqual(len(self.dynamic_cube.get_matrix()), 12)

    def test_get_set_collidable(self):
        self.dynamic_cube.set_collidable(False)
        self.assertFalse(self.dynamic_cube.is_collidable())
        self.dynamic_cube.set_collidable(True)
        self.assertTrue(self.dynamic_cube.is_collidable())

    def test_get_set_measurable(self):
        self.dynamic_cube.set_measurable(False)
        self.assertFalse(self.dynamic_cube.is_measurable())
        self.dynamic_cube.set_measurable(True)
        self.assertTrue(self.dynamic_cube.is_measurable())

    def test_get_set_detectable(self):
        self.dynamic_cube.set_detectable(False)
        self.assertFalse(self.dynamic_cube.is_detectable())
        self.dynamic_cube.set_detectable(True)
        self.assertTrue(self.dynamic_cube.is_detectable())

    def test_get_set_renderable(self):
        self.dynamic_cube.set_renderable(False)
        self.assertFalse(self.dynamic_cube.is_renderable())
        self.dynamic_cube.set_renderable(True)
        self.assertTrue(self.dynamic_cube.is_renderable())

    def test_is_model(self):
        self.assertFalse(self.dynamic_cube.is_model())
        self.assertTrue(self.simple_model.is_model())

    def test_set_model(self):
        self.simple_model.set_model(False)
        self.dynamic_cube.set_model(True)
        self.assertFalse(self.simple_model.is_model())
        self.assertTrue(self.dynamic_cube.is_model())

    def test_remove(self):
        self.dynamic_cube.remove()
        self.simple_model.remove()
        self.assertFalse(self.dynamic_cube.still_exists())
        self.assertFalse(self.simple_model.still_exists())
        self.assertFalse(Object.exists('dynamic_cube'))
        self.assertFalse(Object.exists('simple_model'))

    def test_dynamic_object(self):
        # Can't really test this. So lets just make sure it doesn't error
        self.dynamic_cube.reset_dynamic_object()

    def test_get_bounding_box(self):
        bb = self.dynamic_cube.get_bounding_box()
        self.assertTrue(np.allclose(bb, [-0.05, 0.05] * 3))

    def test_get_objects_in_tree(self):
        dummys = [Dummy('nested_dummy%d' % i) for i in range(3)]
        self.assertListEqual(dummys[0].get_objects_in_tree(
            exclude_base=False, first_generation_only=False), dummys)

        self.assertListEqual(
            dummys[0].get_objects_in_tree(
                exclude_base=True, first_generation_only=False), dummys[1:])
        self.assertListEqual(
            dummys[0].get_objects_in_tree(
                exclude_base=False,first_generation_only=True), dummys[:-1])

    def test_get_extention_string(self):
        self.assertEqual(self.dynamic_cube.get_extension_string(), 'test')

    def test_get_configuration_tree(self):
        config = self.dynamic_cube.get_configuration_tree()
        self.assertIsNotNone(config)

    def test_rotate(self):
        self.dynamic_cube.rotate([0.02, 0.04, 0.06])
        self.assertTrue(np.allclose(
            self.dynamic_cube.get_orientation(), [0.02, 0.04, 0.06]))

    def test_get_set_model_collidable(self):
        self.simple_model.set_model_collidable(False)
        self.assertFalse(self.simple_model.is_model_collidable())
        self.simple_model.set_model_collidable(True)
        self.assertTrue(self.simple_model.is_model_collidable())

    def test_get_set_model_measurable(self):
        self.simple_model.set_model_measurable(False)
        self.assertFalse(self.simple_model.is_model_measurable())
        self.simple_model.set_model_measurable(True)
        self.assertTrue(self.simple_model.is_model_measurable())

    def test_get_set_model_detectable(self):
        self.simple_model.set_model_detectable(False)
        self.assertFalse(self.simple_model.is_model_detectable())
        self.simple_model.set_model_detectable(True)
        self.assertTrue(self.simple_model.is_model_detectable())

    def test_get_set_model_renderable(self):
        self.simple_model.set_model_renderable(False)
        self.assertFalse(self.simple_model.is_model_renderable())
        self.simple_model.set_model_renderable(True)
        self.assertTrue(self.simple_model.is_model_renderable())

    def test_get_set_model_dynamic(self):
        self.simple_model.set_model_dynamic(False)
        self.assertFalse(self.simple_model.is_model_dynamic())
        self.simple_model.set_model_dynamic(True)
        self.assertTrue(self.simple_model.is_model_dynamic())

    def test_get_set_model_respondable(self):
        self.simple_model.set_model_respondable(False)
        self.assertFalse(self.simple_model.is_model_respondable())
        self.simple_model.set_model_respondable(True)
        self.assertTrue(self.simple_model.is_model_respondable())

    def test_check_collision(self):
        c1 = Shape('colliding_cube0')
        c2 = Shape('colliding_cube1')
        self.assertTrue(c1.check_collision(c2))

    def test_check_collision_all(self):
        c1 = Shape('colliding_cube0')
        self.assertTrue(c1.check_collision(None))

    def test_copy(self):
        cube1 = self.cube0.copy()
        self.assertGreater(cube1.get_handle(), 0)
        self.assertIsInstance(cube1, Shape)
        self.assertNotEqual(self.cube0, cube1)

    def test_check_distance(self):
        dist = self.dummy.check_distance(self.cube0)
        self.assertAlmostEqual(dist, 1.4629, places=3)


if __name__ == '__main__':
    unittest.main()
