import pickle

import numpy as np


def main():
    instruction_orders = ["s1up", "s1down", "s1right",
                          "s1left", "s2up", "s2down", "s2right", "s2left"]

    with open("data_orb.pkl", 'rb') as f:
        orb_data = pickle.load(f)

    with open("data_car.pkl", 'rb') as f:
        car_data = pickle.load(f)

    with open("data_ins.pkl", 'rb') as f:
        instructions = pickle.load(f)

    instruction_outputs = [0, 0, 0, 0, 0, 0, 0, 0]

    for i in range(8):
        tmp_sum = 0
        for ins in instructions:
            if ins == instruction_orders[i]:
                tmp_sum += 1
        instruction_outputs[i] = 1 if tmp_sum > 0 else 0

    ret_val = *instruction_outputs, *(orb_data[-1]), *(car_data[-1])
    return tuple([str(v) for v in ret_val])


if __name__ == '__main__':
    data = main()
    print(data)
