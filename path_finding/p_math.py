def sum_nodes(a:tuple, b:tuple) -> tuple:
    """Sums up two pos element wise."""
    return a[0] + b[0], a[1] + b[1]

def diff_nodes(a:tuple, b:tuple) -> tuple:
    """Sums up two pos element wise. -> relative distance between two nodes"""
    return b[0]-a[0], b[1]-a[1]

def sum_abs_nodes(a:tuple, b:tuple) -> tuple:
    """Sums up two absolute pos element wise."""
    a = abs(a[0]),abs(a[1])
    b = abs(b[0]),abs(b[1])
    return a[0] + b[0], a[1] + b[1]

def sum_absolute_a_b(a: int, b: int) -> int:
    """Sums up absolute a and absolute b."""
    a = abs(a)
    b = abs(b)
    return a + b

def diff_absolute_a_b(a:int, b:int) -> int:
    """Sums up absolute a and absolute b."""
    a = abs(a)
    b = abs(b)
    return a - b

