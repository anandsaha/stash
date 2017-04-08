import sys

def convert(s):
    """Convert input to int
    
        Args:
            s: input object

        Returns: integer value after converting the input to s
    """
    try:
        x = int(s)
    except (ValueError, TypeError) as e:
        print("Exception raised: {0}".format(str(e)), file=sys.stderr)
        x = -1
    return x
