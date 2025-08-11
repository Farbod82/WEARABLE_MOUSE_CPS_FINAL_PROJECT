import csv
import math
import matplotlib.pyplot as plt
import sys


def read_tasks_from_csv(filename):
    with open(filename, newline='') as csvfile:
        reader = list(csv.reader(csvfile))
        wcets = list(map(float, reader[0][1:]))
        periods = list(map(float, reader[1][1:]))
        deadlines = list(map(float, reader[2][1:]))

    tasks = []
    for i in range(len(wcets)):
        tasks.append({
            'wcet': wcets[i],
            'period': periods[i],
            'deadline': deadlines[i]
        })

    return tasks


def lcm_list(numbers):
    lcm = numbers[0]
    for number in numbers[1:]:
        lcm = lcm * number // math.gcd(lcm, number)
    return lcm


def find_divisors(n):
    return [i for i in range(1, n + 1) if n % i == 0]


def check_deadline_constraint(f, task):
    return task['deadline'] >= (2 * f - math.gcd(f, int(task['period'])))


def compute_valid_frame_sizes(tasks):
    periods = [int(task['period']) for task in tasks]
    wcets = [task['wcet'] for task in tasks]
    hyperperiod = lcm_list(periods)
    frame_candidates = find_divisors(hyperperiod)

    max_wcet = max(wcets)
    valid_frames = []

    for f in frame_candidates:
        if f < max_wcet:
            continue
        if all(check_deadline_constraint(f, task) for task in tasks):
            valid_frames.append(f)

    return valid_frames


def generate_schedule(tasks, frame_size):
    periods = [int(task['period']) for task in tasks]
    hyperperiod = lcm_list(periods)
    num_frames = hyperperiod // frame_size

    task_instances = []
    for i, task in enumerate(tasks):
        num_instances = hyperperiod // int(task['period'])
        for k in range(num_instances):
            release_time = k * int(task['period'])
            deadline = release_time + task['deadline']
            task_instances.append({
                'task_id': i,
                'instance_id': k,
                'label': f'Task{i+1}[{k}]',
                'release_time': release_time,
                'deadline': deadline,
                'wcet': task['wcet'],
                'scheduled': False
            })

    task_instances.sort(key=lambda x: x['release_time'])

    schedule = [[] for _ in range(num_frames)]

    for frame_index in range(num_frames):
        frame_start = frame_index * frame_size
        frame_end = frame_start + frame_size
        time_cursor = frame_start
        remaining_time = frame_size

        for inst in task_instances:
            if inst['scheduled']:
                continue
            if inst['release_time'] <= frame_start and inst['deadline'] >= frame_end:
                if inst['wcet'] <= remaining_time:
                    schedule[frame_index].append({
                        'label': inst['label'],
                        'start': time_cursor,
                        'duration': inst['wcet']
                    })
                    time_cursor += inst['wcet']
                    remaining_time -= inst['wcet']
                    inst['scheduled'] = True

    return schedule, hyperperiod


def visualize_schedule(schedule, frame_size):
    task_colors = {}
    color_map = plt.get_cmap('tab20')

    task_names = set()
    for frame_tasks in schedule:
        for task in frame_tasks:
            task_name = task['label'].split('[')[0]
            task_names.add(task_name)

    def extract_task_number(name):
        return int(''.join(filter(str.isdigit, name)))

    sorted_task_names = sorted(task_names, key=extract_task_number)
    task_y = {name: i for i, name in enumerate(sorted_task_names)}

    bars = []
    for frame_tasks in schedule:
        for task in frame_tasks:
            task_name = task['label'].split('[')[0]
            y = task_y[task_name]
            bars.append({
                'label': task['label'],
                'start': task['start'],
                'duration': task['duration'],
                'y': y
            })

    fig, ax = plt.subplots(figsize=(12, 0.8 * len(task_y) + 2))

    for i, bar in enumerate(bars):
        color = task_colors.setdefault(bar['label'].split('[')[0], color_map(i % 20))
        ax.broken_barh([(bar['start'], bar['duration'])], (bar['y'], 0.8),
                       facecolors=color, label=bar['label'])
    ax.set_xlabel("Time (micro seconds)")
    ax.set_yticks([y + 0.4 for y in task_y.values()])
    ax.set_yticklabels(sorted_task_names)
    ax.set_title("Static Schedule Gantt Chart")
    ax.grid(True, axis='x', linestyle='--', alpha=0.5)
    ax.set_xlim(0, len(schedule) * frame_size)
    ax.set_ylim(-0.2, len(task_y))

    for x in range(0, len(schedule) * frame_size + 1, frame_size):
        if x % 1000 != 0:
            ax.axvline(x, color='gray', linestyle='--', linewidth=0.8, alpha=0.6)

    ax.set_xticks(range(0, len(schedule) * frame_size + 1, frame_size))

    plt.tight_layout()
    plt.show()


def export_schedule_to_csv(valid_frames, selected_frame_size, schedule, frame_size, output_filename="schedule_output.csv"):
    with open(output_filename, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)

        writer.writerow(["Valid Frame Sizes (ms)"])
        writer.writerow(valid_frames)
        writer.writerow([])

        writer.writerow(["Minimum Valid Frame Size (ms)", min(valid_frames)])
        writer.writerow(["Selected Frame Size (ms)", selected_frame_size])
        writer.writerow([])

        writer.writerow(["Frame Number", "Start Time (ms)", "End Time (ms)", "Scheduled Task Instances"])
        for i, frame_tasks in enumerate(schedule):
            start = i * frame_size
            end = (i + 1) * frame_size
            if frame_tasks:
                task_descriptions = [f"{task['label']} ({task['start']}–{task['start'] + task['duration']} ms)" for task in frame_tasks]
            else:
                task_descriptions = ["Idle"]
            writer.writerow([i + 1, start, end, ", ".join(task_descriptions)])


def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <filename.csv>")
        sys.exit(1)

    filename = sys.argv[1]
    tasks = read_tasks_from_csv(filename)

    valid_frames = compute_valid_frame_sizes(tasks)
    print("Valid frame sizes (ms):", valid_frames)

    if valid_frames:
        selected_frame_size = min(valid_frames)
        print(f"Selected frame size: {selected_frame_size} ms")

        schedule, hyperperiod = generate_schedule(tasks, selected_frame_size)
        print(f"Static schedule for hyperperiod = {hyperperiod} ms:")

        for i, frame_tasks in enumerate(schedule):
            start = i * selected_frame_size
            end = (i + 1) * selected_frame_size
            if frame_tasks:
                descriptions = [f"{task['label']} ({task['start']}–{task['start'] + task['duration']} ms)" for task in frame_tasks]
            else:
                descriptions = ["Idle"]
            print(f"  Frame {i + 1} [{start}–{end} ms]: {', '.join(descriptions)}")

        visualize_schedule(schedule, selected_frame_size)

        export_schedule_to_csv(valid_frames, selected_frame_size, schedule, selected_frame_size)

    else:
        print("No valid frame size found.")


if __name__ == "__main__":
    main()
