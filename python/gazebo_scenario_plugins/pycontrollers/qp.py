import numpy as np
import scipy.sparse
from typing import Tuple, Union


class CostBuilder:

    def __init__(self,
                 num_vars: int,
                 H: Union[np.ndarray, scipy.sparse.csc_matrix] = None,
                 g: np.ndarray = None):

        self.g = g
        self.H = None

        if isinstance(H, scipy.sparse.csc_matrix):
            self.H = H.todense()
        elif isinstance(H, np.ndarray):
            self.H = H
        else:
            if H is not None:
                raise ValueError(f"Type of H variable not recognized ({type(H)})")

        self.num_vars = num_vars

    def add_task(self,
                 A: np.ndarray,
                 a: np.ndarray,
                 weight: float = 1.0) -> "CostBuilder":

        if A.ndim == 1:
            A = A.reshape(1, A.size)

        assert a.ndim == 1
        a = a.reshape(a.size, 1)

        # print(A.shape)
        # print(a.shape)

        assert 1 <= A.ndim <= 2
        assert A.shape[0] == a.shape[0]
        assert A.shape[1] == self.num_vars

        if self.H is None:
            self.H = np.zeros_like(A.transpose().dot(A))

        if self.g is None:
            self.g = np.zeros_like(A.transpose() @ a)

        self.H += weight * A.transpose().dot(A)
        self.g += weight * (-2 * A.transpose() @ a)

        return self

    def build(self) -> Tuple[scipy.sparse.csc_matrix, np.ndarray]:

        return scipy.sparse.csc_matrix(self.H), self.g


class ConstraintsBuilder:

    def __init__(self,
                 num_vars: int,
                 C: np.ndarray = None,
                 l: np.ndarray = None,
                 u: np.ndarray = None):

        self.l = l
        self.u = u
        self.C = C

        self.num_vars = num_vars

    def add_bound(self,
                  C: np.ndarray,
                  l: np.ndarray,
                  u: np.ndarray) -> "ConstraintsBuilder":

        # print("C", C.shape)

        assert C.ndim == 2
        assert l.ndim == 1 and u.ndim == 1
        assert C.shape[0] == l.shape[0] == u.shape[0]
        assert C.shape[1] == self.num_vars

        if self.C is None:
            self.C = C
        else:
            self.C = np.concatenate([self.C, C])

        if self.l is None:
            # self.l = l.reshape(1, l.size)
            self.l = l
        else:
            # self.l = np.concatenate([self.l, l.reshape(1, l.size)])
            self.l = np.concatenate([self.l, l])

        if self.u is None:
            # self.u = u.reshape(1, u.size)
            self.u = u
        else:
            # self.u = np.concatenate([self.u, u.reshape(1, u.size)])
            self.u = np.concatenate([self.u, u])

        return self

    def add_equality(self, C: np.ndarray, b: np.ndarray) -> "ConstraintsBuilder":

        return self.add_bound(C=C, l=b, u=b)

    def add_lower_bound(self, C: np.ndarray, l: np.ndarray) -> "ConstraintsBuilder":

        return self.add_bound(C=C, l=l, u=np.inf*np.ones_like(l))

    def add_upper_bound(self, C: np.ndarray, u: np.ndarray) -> "ConstraintsBuilder":

        # print(u)
        # print(-np.inf*np.ones_like(u))
        # print(C)

        return self.add_bound(C=C, l=-np.inf*np.ones_like(u), u=u)

    def build(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:

        return self.l, self.u, self.C
