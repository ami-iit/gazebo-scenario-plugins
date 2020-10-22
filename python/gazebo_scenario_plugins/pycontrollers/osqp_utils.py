import enum
import numpy as np
from dataclasses import dataclass


@dataclass
class OSQPResult:

    x: np.ndarray
    y: np.ndarray
    prim_inf_cert: bool
    dual_inf_cert: bool
    info: None  # TODO

    @staticmethod
    def Build(result) -> "OSQPResult":

        return OSQPResult(x=result.x,
                          y=result.y,
                          prim_inf_cert=result.prim_inf_cert,
                          dual_inf_cert=result.dual_inf_cert,
                          info=result.info)


class OSQPStatus(enum.Enum):

    OSQP_SOLVED = 1
    OSQP_SOLVED_INACCURATE = 2
    OSQP_MAX_ITER_REACHED = -2
    OSQP_PRIMAL_INFEASIBLE = -3
    OSQP_PRIMAL_INFEASIBLE_INACCURATE = 3
    OSQP_DUAL_INFEASIBLE = -4
    OSQP_DUAL_INFEASIBLE_INACCURATE = 4
    OSQP_SIGINT = -5
    OSQP_TIME_LIMIT_REACHED = -6
    OSQP_UNSOLVED = -10
    OSQP_NON_CVX = -7
