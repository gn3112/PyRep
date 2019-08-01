from typing import List, Tuple
from pyrep.backend import vrep
from pyrep.objects.object import Object
from pyrep.const import ObjectType, PrimitiveShape, TextureMappingMode
from pyrep.textures.texture import Texture
import os

class Light(Object):

    def set_parameters(self, state: bool, diffusePart: list, specularPart: list) -> None:
        vrep.simSetLightParameters(self._handle, state, diffusePart, specularPart)

    def get_parameters(self) -> List:
        return vrep.simGetLightParameters(self._handle)

    def get_type(self) -> ObjectType:
        return ObjectType.LIGHT
