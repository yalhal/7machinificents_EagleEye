function [p_eci] = CalcIntersectPoint(utc, r_eci, q, line_body)
    global r_earth_
    dcm = q2dcm(q)';
    line_eci = dcm * line_body;
    b = dot(r_eci, line_eci);
    c = norm(r_eci) ^ 2 - r_earth_ ^ 2;
    s = -b - sqrt(b^2 - c);
    p_eci = r_eci + s * line_eci;
end
