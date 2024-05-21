"""
   Border

   Structure for storing the borders of an area in a two dimensional euclidean space in.
"""
struct Border
    left::Int64
    right::Int64
    bottom::Int64
    top::Int64

    Border(left, right, bottom, top) = left >= right || bottom >= top ? error("incorrect border") : new(left, right, bottom, top)
end


"""
ratio(border::Border) -> Float64

Calculates the ratio of width to height of the area.

# Arguments
- `border::Border`: Border to calculate the ration of

# Returns
- `Float64`: Ratio
"""
function ratio(border::Border)
    return (border.top - border.bottom)/(border.right - border.left)
end

"""
x_axis(border::Border) -> Tuple{Int64, Int64}

Returns the borders on the x-axis as Tuple

# Arguments
- `border::Border`: Border to return the values from
"""
function x_axis(border::Border)
    return (border.left, border.right)
end

"""
y_axis(border::Border) -> Tuple{Int64, Int64}

Returns the borders on the y-axis as Tuple

# Arguments
- `border::Border`: Border to return the values from
"""
function y_axis(border::Border)
    return (border.bottom, border.top)
end