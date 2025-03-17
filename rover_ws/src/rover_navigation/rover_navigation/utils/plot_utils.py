import matplotlib.pyplot as plt


def plotOrder(order, waypoints, fix):
    """
    Plot the order of task legs using matplotlib
    """

    plt.title("Order Planner")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")

    # Plot from current fix to the first leg
    plt.text(fix.position.longitude, fix.position.latitude, "FIX")
    for wp in waypoints:
        if wp["leg"] == order[0]:
            first = wp
            plt.text(first["longitude"], first["latitude"], first["leg"])
    plt.plot(
        [fix.position.longitude, first["longitude"]],
        [fix.position.latitude, first["latitude"]],
        "ro-",
    )

    # Plot the rest of the legs
    for i in range(len(order) - 1):
        for wp in waypoints:
            if wp["leg"] == order[i]:
                start = wp
                plt.text(start["longitude"], start["latitude"], start["leg"])
            elif wp["leg"] == order[i + 1]:
                end = wp
                plt.text(end["longitude"], end["latitude"], end["leg"])
        plt.plot(
            [start["longitude"], end["longitude"]],
            [start["latitude"], end["latitude"]],
            "ro-",
        )
    plt.show()
