"""
데이터베이스 연습 #326
SQLite CRUD 실습
"""
import sqlite3

def setup_326():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_326 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_326_0', 233.6, 'C'), ('item_326_1', 829.78, 'A'), ('item_326_2', 657.2, 'C'), ('item_326_3', 761.99, 'A'), ('item_326_4', 411.23, 'B'), ('item_326_5', 557.17, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_326 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_326()
    cursor = conn.cursor()

    print(f"DB 연습 #326")
    cursor.execute('SELECT * FROM items_326')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_326 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
