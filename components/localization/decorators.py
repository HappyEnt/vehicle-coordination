"""
This file holds some function decorators, which are useful for the evaluation.
"""
from time import time
from os import path

dirname = path.dirname(__file__)


def measure_time(
    do_print=False,
    write_to_file=False,
    filename=dirname + "/data.csv",
    csv_separator="&",
    with_line_break=True,
):
    """A decorator to measure the execution time of a function."""

    def decorator(function):
        def wrapper(*args, **kwargs):
            start = time()
            result = function(*args, **kwargs)
            elapsed = time() - start
            if do_print:
                print(
                    "Time elapsed for function "
                    + str(function.__name__)
                    + ": "
                    + str(elapsed)
                    + " seconds"
                )
            if write_to_file:
                with  open(filename, "a", encoding="utf-16") as f:
                    end_string = str(function.__name__) + csv_separator + str(elapsed)
                    if with_line_break:
                        end_string = end_string + "\n"
                    f.write(end_string)
            return result

        return wrapper

    return decorator
