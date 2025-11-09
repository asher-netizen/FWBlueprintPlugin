using System;

namespace FWBlueprintPlugin.Services
{
    internal static class DimensionFormatting
    {
        public static string FormatDimension(double inches, int denominatorPower)
        {
            int wholeInches = (int)Math.Floor(inches);
            double fraction = inches - wholeInches;

            int denominator = (int)Math.Pow(2, denominatorPower);
            int numerator = (int)Math.Round(fraction * denominator);

            if (numerator == 0)
            {
                return $"{wholeInches}\"";
            }

            if (numerator == denominator)
            {
                return $"{wholeInches + 1}\"";
            }

            int gcd = GreatestCommonDivisor(numerator, denominator);
            numerator /= gcd;
            denominator /= gcd;

            if (wholeInches == 0)
            {
                return $"{numerator}/{denominator}\"";
            }

            return $"{wholeInches}-{numerator}/{denominator}\"";
        }

        private static int GreatestCommonDivisor(int a, int b)
        {
            while (b != 0)
            {
                int temp = b;
                b = a % b;
                a = temp;
            }

            return a;
        }
    }
}
