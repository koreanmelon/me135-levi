import pickle


def main():
    with open("data_orb.pkl", 'rb') as f:
        orb_data = pickle.load(f)

    with open("data_car.pkl", 'rb') as f:
        car_data = pickle.load(f)

    with open("data_ins.pkl", 'rb') as f:
        instructions = pickle.load(f)

    return orb_data, car_data, instructions


if __name__ == '__main__':
    data = main()
    print(f"Orb data: {data[0][-1]}")
    print(f"Car data: {data[1][-1]}")
    print(f"Instructions: {data[2]}")
