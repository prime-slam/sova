from typing import Annotated, Literal, TypeVar
import numpy.typing as npt
import numpy as np

__all__ = [
    "Array3",
    "ArrayNx3",
    "ArrayNx4",
    "ArrayNx8",
]

DType = TypeVar("DType", bound=np.generic)

Array3 = Annotated[npt.NDArray[DType], Literal[3]]

ArrayNx3 = Annotated[npt.NDArray[DType], Literal["N", 3]]

ArrayNx4 = Annotated[npt.NDArray[DType], Literal["N", 4]]

ArrayNx8 = Annotated[npt.NDArray[DType], Literal["N", 8]]
