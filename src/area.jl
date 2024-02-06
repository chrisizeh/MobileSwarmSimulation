
struct Border
    left::Int64
    right::Int64
    bottom::Int64
    top::Int64

    Border(left, right, bottom, top) = left >= right || bottom >= top ? error("incorrect border") : new(left, right, bottom, top)
end

function x_axis(border::Border)
    return (border.left, border.right)
end

function y_axis(border::Border)
    return (border.bottom, border.top)
end