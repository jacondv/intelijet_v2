import logging
from functools import wraps

import grpc


def none_if_not_connected(func):

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_connected():
            logging.warning("Please connect first.")
            return None
        return func(self, *args, **kwargs)

    return wrapper


def false_if_not_connected(func):

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_connected():
            logging.warning("Please connect first.")
            return False
        return func(self, *args, **kwargs)

    return wrapper


def log_rpc_error(error_msg: grpc.RpcError) -> None:
    # Error returned by the gRPC service
    logging.error(f"RPC-Error occurred with error message: {error_msg.details()}.")


def none_if_rpc_error(func):

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except grpc.RpcError as error_msg:
            log_rpc_error(error_msg)
            return None

    return wrapper


def false_if_rpc_error(func):

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except grpc.RpcError as error_msg:
            log_rpc_error(error_msg)
            return False

    return wrapper
