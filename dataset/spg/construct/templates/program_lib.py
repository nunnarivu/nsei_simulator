


PROGRAM_LIB = dict()


PROGRAM_LIB['row'] = [("length", int), ("objects", list[int]),
                    "def row(length: int, objects: list[int]) -> None: \
                        \n\tplace_at_focus(objects.pop(0))\
                        \n\tfor i in range(length-1): \
                            \n\t\tshift_focus('right') \
                            \n\t\tplace_at_focus(objects.pop(0))"]

PROGRAM_LIB['column'] = [("length", int), ("objects", list[int]),
                        "def column(length: int, objects: list[int]) -> None: \
                            \n\tplace_at_focus(objects.pop(0))\
                            \n\tfor i in range(length-1): \
                                \n\t\tshift_focus('back') \
                                \n\t\tplace_at_focus(objects.pop(0))"]

PROGRAM_LIB['tower'] = [("height", int), ("objects", list[int]),
                        "def tower(height: int, objects: list[int]) -> None: \
                            \n\tplace_at_focus(objects.pop(0))\
                            \n\tfor i in range(height-1): \
                                \n\t\tshift_focus('top') \
                                \n\t\tplace_at_focus(objects.pop(0))"]