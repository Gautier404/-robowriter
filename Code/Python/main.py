from SVG_to_coords import *
from coords_to_toolpath import *
from generate_toolpath import *
from fit_path import *
from animate_arm import *
import plotly.graph_objects as go

current_working_directory = os.getcwd()
file_name = input("What file do you want to use?\n")
svg_path = "\\Code\\Python\\svgs\\"
#file_name = "rectangle"
cwd_split = current_working_directory.split('\\')
#print(cwd_split[len(cwd_split)-1])
if cwd_split[len(cwd_split)-1] == "robowriter":
    file_path = current_working_directory + svg_path + file_name + '.svg'
else:
    file_path = current_working_directory + '\\robowriter' + svg_path + file_name + '.svg'
#print(parse_svg(file_path))
#convert_to_toolpath(paths)
coords = parse_svg(file_path)
toolpath = coords_to_toolpath(coords)
for i in range(0, len(toolpath)):
    for j in range(0, len(toolpath[i])):
        toolpath[i][j][0] = float(toolpath[i][j][0])
        toolpath[i][j][1] = float(toolpath[i][j][1])
print(toolpath)

bounds = np.array([[100, 150], [200, -150]], dtype=float)
fitted_toolpath = fit_path(toolpath, bounds)
consolidated_toolpath = np.concatenate(toolpath)
consolidated_fitted_toolpath = np.concatenate(fitted_toolpath)

# Create figure
fig = go.Figure()

# Plot original toolpath
fig.add_trace(go.Scatter(x=consolidated_toolpath[:,0], y=consolidated_toolpath[:,1], mode='lines', name='Original Toolpath'))

# Plot bounds
fig.add_trace(go.Scatter(x=bounds[:,0], y=bounds[:,1], mode='markers', name='Bounds'))

# Plot scaled toolpath
fig.add_trace(go.Scatter(x=consolidated_fitted_toolpath[:,0], y=consolidated_fitted_toolpath[:,1], mode='lines', name='Scaled Toolpath'))

# Set layout
fig.update_layout(title='Toolpath Scaling',
                    showlegend=True)

# Show plot
fig.show()

cartesian_toolpath = generate_cartesian_toolpath(fitted_toolpath)

fig_2 = go.Figure()
fig_2.add_trace(go.Scatter3d(x=cartesian_toolpath[:,0], y=cartesian_toolpath[:,1], z=cartesian_toolpath[:,2], mode='markers', name = 'Interpolated Toolpath', marker=dict(color="blue", size=2)))
#fig.add_trace(go.Scatter3d(x=consolidated_toolpath[:,0], y=consolidated_toolpath[:,1], z=np.zeros(consolidated_toolpath.shape[0]), mode='markers', name = 'Original Toolpath'))
fig_2.show()

angular_toolpath = generate_angular_toolpath(cartesian_toolpath)
animate_arm(angular_toolpath, cartesian_toolpath)
