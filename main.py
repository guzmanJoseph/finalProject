from UFGraph import UFGraph
import streamlit as st


def main():
    # set up webpage
    st.set_page_config(page_title="Location Selector", layout="centered")

    # style of webpage
    st.markdown(
        """
        <style>
        .stApp {
            background-color: white; /* pure white background */
        }
        .block-container {
            padding-top: 2rem;
        }
        .dropdown-style {
            border: 2px solid #7c663f;
            padding: 0.5em;
            margin-bottom: 1.5em;
        }
        </style>
        """,
        unsafe_allow_html=True
    )

    # title of webpage
    st.markdown("## Enter initial and end location:")

    # list of location options
    locations = [
        "Library West", "Marston Science Library", "Gator Corner Dining", "Newell Hall",
        "Norman Hall", "Pugh Hall", "New Physics Building", "Little Hall",
        "Weil Hall", "Matherly Hall", "Heavener Hall", "Hough Hall", "Anderson Hall", "Leigh Hall", "Grinter Hall",
        "Frazier Rogers Hall", "Floyd Hall", "Fine Arts Building A", "Fine Arts Building C",
        "Nanoscale Research Facility", "Biomedical Sciences Building", "Shands Hospital",
        "Ben Hill Griffin Stadium", "Stephen C. O'Connell Center", "Condron Family Ballpark","Beaty Towers", "Hume Hall", "Jennings Hall", "Broward Hall", "Rawlings Hall", "Graham Hall", "Keys Complex",
        "Southwest Recreation Center", "Student Recreation & Fitness Center", "Bat Houses"
    ]

    algorithms = ["Dijkstra's Algorithm", "A-Star"]

    # dropdown options
    start = st.selectbox("Start location:", locations, key="start", help="Select the starting point")
    end = st.selectbox("End location:", locations, key="end", help="Select the destination")
    algorithm = st.selectbox("Which algorithm would you like to use?", algorithms, key="algo")

    # display selected result
    if start and end and algorithm:
        if start != end:
            st.success(f"You selected to go from **{start}** to **{end}** using **{algorithm}**.")
        else:
            st.warning("Start and end locations are the same. Please select different locations.")
        if st.button("Go"):
            uf = UFGraph()
            graph = uf.makeGraph(uf.osmdata)

            location_to_vertex = {
                "Marston Science Library": 137,
                "Library West": 491,
                "Reitz Union": 31,
                "Gator Corner Dining": 1630,
                "Newell Hall": 661,
                "Norman Hall": 1126,
                "Pugh Hall": 97,
                "New Physics Building": 67,
                "Little Hall": 366,
                "Weil Hall": 1634,
                "Matherly Hall": 566,
                "Heavener Hall": 347,
                "Hough Hall": 1636,
                "Leigh Hall": 1563,
                "Grinter Hall": 968,
                "Frazier Rogers Hall": 527,
                "Floyd Hall": 122,
                "Fine Arts Building A": 366,
                "Fine Arts Building C": 1638,
                "Nanoscale Research Facility": 254,
                "Biomedical Sciences Building": 253,
                "Shands Hospital": 130,
                "Ben Hill Griffin Stadium": 2,
                "Stephen C. O’Connell Center": 1022,
                "Condron Family Ballpark": 994,
                "Beaty Towers": 478,
                "Hume Hall": 483,
                "Jennings Hall": 1664,
                "Broward Hall": 1080,
                "Rawlings Hall": 453,
                "Graham Hall": 678,
                "Keys Complex": 714,
                "Southwest Recreation Center": 738,
                "Student Recreation & Fitness Center": 105,
                "Bat Houses": 802
            }

            # set start and end vertices
            start_vertex_id = location_to_vertex[start]
            end_vertex_id = location_to_vertex[end]
            selected_algorithm = algorithm

            startingVertex = start_vertex_id
            endingVertex = end_vertex_id

            if selected_algorithm == "Dijkstra's Algorithm":
                path = uf.dijkstrasAlgorithm(graph, startingVertex, endingVertex, uf.osmdata)
            else:
                path = uf.AStarAlgorithm(graph, startingVertex, endingVertex, uf.osmdata)

            uf.colorPath(graph, path)
            uf.bridges.visualize()
            url = uf.bridges.get_visualize_url()
            st.markdown(f"[View Graph on Bridges]({url})", unsafe_allow_html=True)

if __name__ == "__main__":
    main()
