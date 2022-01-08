import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

abstract class InputProvider<T> extends StatelessWidget {
  const InputProvider({required this.onSubmit, Key? key}) : super(key: key);

  final void Function(T result) onSubmit;
}

class TextProvider extends InputProvider<String> {
  const TextProvider(
      {required void Function(String) onSubmit, this.hintText, Key? key})
      : super(onSubmit: onSubmit, key: key);

  final String? hintText;

  @override
  Widget build(BuildContext context) {
    return Flexible(
      fit: FlexFit.tight,
      child: TextField(
          maxLines: 1,
          decoration: InputDecoration(
            border: InputBorder.none,
            hintText: hintText ?? 'Here goes your Text',
          ),
          onSubmitted: (String text) => onSubmit(text)),
    );
  }
}

class NumberProvider extends InputProvider<int> {
  const NumberProvider({required void Function(int) onSubmit, Key? key})
      : super(onSubmit: onSubmit, key: key);

  @override
  Widget build(BuildContext context) {
    return Flexible(
      fit: FlexFit.tight,
      child: TextField(
          keyboardType: TextInputType.number,
          inputFormatters: <TextInputFormatter>[
            FilteringTextInputFormatter.digitsOnly
          ],
          maxLines: 1,
          decoration: const InputDecoration(
            border: InputBorder.none,
          ),
          onSubmitted: (String text) => onSubmit(int.parse(text))),
    );
  }
}

class CappedValueProvider extends InputProvider<String> {
  const CappedValueProvider(
      {required this.possibleValues,
      required void Function(String) onSubmit,
      Key? key})
      : super(onSubmit: onSubmit, key: key);

  final List<String> possibleValues;

  @override
  Widget build(BuildContext context) {
    return PopupMenuButton<String>(
      tooltip: 'Select one of ${possibleValues.length} Values',
      onSelected: (String result) => onSubmit(result),
      itemBuilder: (BuildContext context) => <PopupMenuEntry<String>>[
        for (String value in possibleValues)
          PopupMenuItem<String>(child: Text(value), value: value)
      ],
    );
  }
}
