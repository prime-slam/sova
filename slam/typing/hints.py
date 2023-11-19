import numpy as np
import numpy.typing as npt

from typing import Annotated, Literal, TypeVar

__all__ = [
    "Array3",
    "Array4x4",
    "ArrayNx3",
    "ArrayNx4",
    "ArrayNx8",
    "ArrayNx4x4",
]

DType = TypeVar("DType", bound=np.generic)

Array3 = Annotated[npt.NDArray[DType], Literal[3]]

Array4x4 = Annotated[npt.NDArray[DType], Literal[4, 4]]

ArrayNx3 = Annotated[npt.NDArray[DType], Literal["N", 3]]

ArrayNx4 = Annotated[npt.NDArray[DType], Literal["N", 4]]

ArrayNx8 = Annotated[npt.NDArray[DType], Literal["N", 8]]

ArrayNx4x4 = Annotated[npt.NDArray[DType], Literal["N", 4, 4]]
